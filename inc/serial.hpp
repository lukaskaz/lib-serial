#include <termios.h>
#include <unistd.h>

#include <string>
#include <vector>

enum class debug_t
{
    nodebug,
    usedebug
};

class serial
{
  public:
    virtual ~serial()
    {}

    virtual size_t read(std::vector<uint8_t>&, ssize_t, uint32_t, debug_t) = 0;
    virtual size_t read(std::vector<uint8_t>&, ssize_t, debug_t) = 0;
    virtual size_t write(const std::vector<uint8_t>&, debug_t) = 0;
    virtual void flushBuffer() = 0;

    size_t read(std::vector<uint8_t>& vect, ssize_t size)
    {
        return read(vect, size, debug_t::nodebug);
    }
    size_t read(std::vector<uint8_t>& vect, ssize_t size, uint32_t timeout)
    {
        return read(vect, size, timeout, debug_t::nodebug);
    }
    size_t write(const std::vector<uint8_t>& vect)
    {
        return write(vect, debug_t::nodebug);
    }

  protected:
    void showserialtraces(std::string_view, const std::vector<uint8_t>&,
                          debug_t);
};

class uart : public serial
{
  public:
    explicit uart(const std::string&, speed_t);
    ~uart();

    size_t read(std::vector<uint8_t>&, ssize_t, uint32_t, debug_t) override;
    size_t read(std::vector<uint8_t>&, ssize_t, debug_t) override;
    size_t write(const std::vector<uint8_t>&, debug_t) override;
    void flushBuffer() override;

  private:
    const int32_t fd;

    void configure(speed_t);
};

class usb : public serial
{
  public:
    explicit usb(const std::string&, speed_t);
    ~usb();

    size_t read(std::vector<uint8_t>&, ssize_t, uint32_t, debug_t) override;
    size_t read(std::vector<uint8_t>&, ssize_t, debug_t) override;
    size_t write(const std::vector<uint8_t>&, debug_t) override;
    void flushBuffer() override;

  private:
    const int32_t fd;

    void disableFlowControl();
    ssize_t bytesInBuffer();
    void configure(speed_t);
};
