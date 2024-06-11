#include <termios.h>
#include <unistd.h>

#include <string>
#include <vector>

class serial
{
  public:
    virtual ~serial()
    {}

    virtual size_t read(std::vector<uint8_t>&, ssize_t, uint32_t, bool) = 0;
    virtual size_t read(std::vector<uint8_t>&, ssize_t, bool) = 0;
    virtual size_t write(const std::vector<uint8_t>&, bool) = 0;
    virtual void flushBuffer() = 0;
};

class uart : public serial
{
  public:
    explicit uart(const std::string&, speed_t);
    ~uart();

    size_t read(std::vector<uint8_t>&, ssize_t, uint32_t, bool) override;
    size_t read(std::vector<uint8_t>&, ssize_t, bool) override;
    size_t write(const std::vector<uint8_t>&, bool) override;
    void flushBuffer() override;

  private:
    const int32_t fd;

    void configure(speed_t);
    void showserialtraces(std::string_view, const std::vector<uint8_t>&, bool);
};

class usb : public serial
{
  public:
    explicit usb(const std::string&, speed_t);
    ~usb();

    size_t read(std::vector<uint8_t>&, ssize_t, uint32_t, bool) override;
    size_t read(std::vector<uint8_t>&, ssize_t, bool) override;
    size_t write(const std::vector<uint8_t>&, bool) override;
    void flushBuffer() override;

  private:
    const int32_t fd;

    void disableFlowControl();
    uint32_t bytesInBuffer();
    void configure(speed_t);
    void showserialtraces(std::string_view, const std::vector<uint8_t>&, bool);
};
