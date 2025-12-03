//! DMA Ring Buffer support
//!
//! This module provides ring buffer abstractions for DMA circular mode transfers.
//! Ported from embassy-stm32.

use core::future::poll_fn;
use core::task::{Poll, Waker};

use super::word::Word;

/// DMA controller trait for ring buffer operations
pub trait DmaCtrl {
    /// Get the remaining transfer count (NDTR equivalent)
    fn get_remaining_transfers(&self) -> usize;

    /// Reset the transfer completed counter to 0 and return the previous value
    fn reset_complete_count(&mut self) -> usize;

    /// Set the waker for async operations
    fn set_waker(&mut self, waker: &Waker);
}

/// Ring buffer error types
#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// DMA overrun - data was lost
    Overrun,
    /// DMA position tracking became inconsistent
    DmaUnsynced,
}

#[derive(Debug, Clone, Copy, Default)]
struct DmaIndex {
    complete_count: usize,
    pos: usize,
}

impl DmaIndex {
    fn reset(&mut self) {
        self.pos = 0;
        self.complete_count = 0;
    }

    fn as_index(&self, cap: usize, offset: usize) -> usize {
        (self.pos + offset) % cap
    }

    fn dma_sync(&mut self, cap: usize, dma: &mut impl DmaCtrl) {
        // HPM DMA circular mode: TC interrupt may fire on wrap-around.
        // But we also detect wrap-around via position comparison.
        // To avoid double-counting, we ONLY use position comparison.
        // Just clear the TC count but don't use it.
        let _count_diff = dma.reset_complete_count();
        let new_pos = cap - dma.get_remaining_transfers();

        // Detect wrap-around: if new_pos < old_pos, DMA has wrapped
        if new_pos < self.pos {
            self.complete_count += 1;
        }
        self.pos = new_pos;
    }

    fn advance(&mut self, cap: usize, steps: usize) {
        let next = self.pos + steps;
        self.complete_count += next / cap;
        self.pos = next % cap;
    }

    fn normalize(lhs: &mut DmaIndex, rhs: &mut DmaIndex) {
        let min_count = lhs.complete_count.min(rhs.complete_count);
        lhs.complete_count -= min_count;
        rhs.complete_count -= min_count;
    }

    fn diff(&self, cap: usize, rhs: &DmaIndex) -> isize {
        (self.complete_count * cap + self.pos) as isize - (rhs.complete_count * cap + rhs.pos) as isize
    }
}

/// Readable DMA ring buffer (peripheral to memory)
pub struct ReadableDmaRingBuffer<'a, W: Word> {
    dma_buf: &'a mut [W],
    write_index: DmaIndex,
    read_index: DmaIndex,
}

impl<'a, W: Word> ReadableDmaRingBuffer<'a, W> {
    /// Create a new ring buffer
    pub fn new(dma_buf: &'a mut [W]) -> Self {
        Self {
            dma_buf,
            write_index: Default::default(),
            read_index: Default::default(),
        }
    }

    /// Reset the ring buffer to initial state
    pub fn reset(&mut self, dma: &mut impl DmaCtrl) {
        dma.reset_complete_count();
        self.write_index.reset();
        self.write_index.dma_sync(self.cap(), dma);
        self.read_index = self.write_index;
    }

    /// Get the buffer capacity
    pub const fn cap(&self) -> usize {
        self.dma_buf.len()
    }

    /// Get the number of readable samples
    pub fn len(&mut self, dma: &mut impl DmaCtrl) -> Result<usize, Error> {
        self.write_index.dma_sync(self.cap(), dma);
        DmaIndex::normalize(&mut self.write_index, &mut self.read_index);

        let diff = self.write_index.diff(self.cap(), &self.read_index);

        if diff < 0 {
            Err(Error::DmaUnsynced)
        } else if diff > self.cap() as isize {
            Err(Error::Overrun)
        } else {
            Ok(diff as usize)
        }
    }

    /// Read elements from the ring buffer
    ///
    /// Returns (bytes_read, bytes_remaining)
    pub fn read(&mut self, dma: &mut impl DmaCtrl, buf: &mut [W]) -> Result<(usize, usize), Error> {
        self.read_raw(dma, buf).inspect_err(|_| {
            self.reset(dma);
        })
    }

    /// Read an exact number of elements (async)
    pub async fn read_exact(&mut self, dma: &mut impl DmaCtrl, buffer: &mut [W]) -> Result<usize, Error> {
        let mut read_data = 0;
        let buffer_len = buffer.len();

        poll_fn(|cx| {
            dma.set_waker(cx.waker());

            match self.read(dma, &mut buffer[read_data..buffer_len]) {
                Ok((len, remaining)) => {
                    read_data += len;
                    if read_data == buffer_len {
                        Poll::Ready(Ok(remaining))
                    } else {
                        Poll::Pending
                    }
                }
                Err(e) => Poll::Ready(Err(e)),
            }
        })
        .await
    }

    fn read_raw(&mut self, dma: &mut impl DmaCtrl, buf: &mut [W]) -> Result<(usize, usize), Error> {
        let readable = self.len(dma)?.min(buf.len());
        for i in 0..readable {
            buf[i] = self.read_buf(i);
        }
        let available = self.len(dma)?;
        self.read_index.advance(self.cap(), readable);
        Ok((readable, available - readable))
    }

    fn read_buf(&self, offset: usize) -> W {
        unsafe { core::ptr::read_volatile(self.dma_buf.as_ptr().add(self.read_index.as_index(self.cap(), offset))) }
    }
}
