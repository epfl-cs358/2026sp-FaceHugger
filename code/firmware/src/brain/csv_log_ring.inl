#ifndef CSV_LOG_RING_INL
#define CSV_LOG_RING_INL

#include <cstring>

namespace csv_log {

template <std::size_t N>
void Ring<N>::append(const char* row, std::size_t row_len) {
    if (row_len == 0 || row_len > N) return;

    // Evict oldest whole rows until row_len fits.
    while (len_ + row_len > N) {
        // Advance tail to the byte after the next '\n', evicting one logical row.
        std::size_t evicted = 0;
        while (evicted < len_) {
            char c = buf_[tail_];
            tail_ = (tail_ + 1) % N;
            ++evicted;
            if (c == '\n') break;
        }
        len_ -= evicted;
        if (evicted == 0) break;  // structural invariant: outer while exits at len_==0
    }

    // Copy row, handling wrap.
    std::size_t first = N - head_;
    if (first >= row_len) {
        std::memcpy(buf_ + head_, row, row_len);
    } else {
        std::memcpy(buf_ + head_, row, first);
        std::memcpy(buf_, row + first, row_len - first);
    }
    head_ = (head_ + row_len) % N;
    len_ += row_len;
}

template <std::size_t N>
void Ring<N>::view(const char*& data1, std::size_t& size1,
                   const char*& data2, std::size_t& size2) const {
    if (len_ == 0) {
        data1 = nullptr; size1 = 0;
        data2 = nullptr; size2 = 0;
        return;
    }
    if (tail_ + len_ <= N) {
        data1 = buf_ + tail_; size1 = len_;
        data2 = nullptr;      size2 = 0;
    } else {
        data1 = buf_ + tail_;            size1 = N - tail_;
        data2 = buf_;                    size2 = len_ - size1;
    }
}

} // namespace csv_log

#endif
