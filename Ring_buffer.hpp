#ifndef HRK_RING_BUFFER_HPP
#define HRK_RING_BUFFER_HPP
#include <cstddef>
#include <deque>
#include <algorithm>
#include <cmath>
namespace hrk
{
    template <class T>
    class Ring_buffer
    {
    public:
        Ring_buffer(void)
        {
        }
        size_t size(void) const
        {
            return ring_buffer_.size();
        }
        bool empty(void) const
        {
            return ring_buffer_.empty();
        }
        size_t push(const T* data, size_t size)
        {
            const T* last_p = data + size;
            ring_buffer_.insert(ring_buffer_.end(), data, last_p);
            return size;
        }
        size_t pop(T* data, size_t size)
        {
            size_t n = min(size,ring_buffer_.size());
            std::copy(ring_buffer_.begin(), ring_buffer_.begin() + n, data);
            ring_buffer_.erase(ring_buffer_.begin(), ring_buffer_.begin() + n);
            return n;
        }
        void ungetc(const T ch)
        {
            ring_buffer_.push_front(ch);
        }
        void clear(void)
        {
            ring_buffer_.clear();
        }
    private:
        Ring_buffer(const Ring_buffer& rhs);
        Ring_buffer& operator = (const Ring_buffer& rhs);

        std::deque<T> ring_buffer_;
    };
}
#endif