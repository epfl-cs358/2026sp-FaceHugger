#ifndef CSV_LOG_H
#define CSV_LOG_H

#include <cstddef>
#include <cstdint>

namespace csv_log {

// State snapshot used to format one CSV row. POD — no Arduino types.
struct State {
    uint32_t millis;
    uint8_t  robot_state;
    uint8_t  gait;
    uint8_t  is_moving;
    uint8_t  is_inverted;
    uint32_t last_cmd_ms;
    float    target_x;
    float    target_y;
    float    target_yaw;
    float    active_x;
    float    active_y;
    float    active_yaw;
    // Order: FR(hip,th,kn), FL(hip,th,kn), BR(hip,th,kn), BL(hip,th,kn)
    float    servo_angles[12];
};

constexpr std::size_t kColumnCount = 24;

// Returns a pointer to the static null-terminated header line, ending with '\n'.
const char* header();

// Formats one CSV row from `s` into `out` (including trailing '\n').
// Returns bytes written (not counting the null terminator), or 0 if `cap` is too small.
std::size_t format_row(const State& s, char* out, std::size_t cap);

// Maximum bytes any row will ever produce (upper bound for buffer sizing).
constexpr std::size_t kMaxRowBytes = 192;

// Append-only ring of whole CSV rows. Static, fixed capacity at compile time.
template <std::size_t N>
class Ring {
public:
    Ring() : head_(0), tail_(0), len_(0) {}
    static constexpr std::size_t capacity() { return N; }
    std::size_t size() const { return len_; }
    void clear() { head_ = tail_ = len_ = 0; }

    // Append a single row. `row` must end with '\n' (the convention of format_row()).
    // If the row is larger than capacity, the call is a no-op. Otherwise oldest whole
    // rows are evicted (tail advances past their trailing '\n') until it fits.
    void append(const char* row, std::size_t row_len);

    // Two-segment view of stored bytes in logical order: [data1, data1+size1) then
    // [data2, data2+size2). Either segment may be empty.
    void view(const char*& data1, std::size_t& size1,
              const char*& data2, std::size_t& size2) const;

private:
    char        buf_[N];
    std::size_t head_;  // next write index
    std::size_t tail_;  // oldest byte
    std::size_t len_;   // bytes in use
};

} // namespace csv_log

// Pull in the template definition.
#include "csv_log_ring.inl"

#endif
