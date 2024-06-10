#include <termios.h>

#include <string>
#include <vector>

class serial
{
  public:
    virtual ~serial()
    {}

    virtual size_t read(std::vector<uint8_t>&, uint32_t, uint32_t) = 0;
    virtual size_t read(std::vector<uint8_t>&, uint32_t) = 0;
    virtual size_t write(const std::vector<uint8_t>&) = 0;
    virtual void flushBuffer() = 0;
};

class uart : public serial
{
  public:
    explicit uart(const std::string&, speed_t);
    ~uart();

    size_t read(std::vector<uint8_t>&, uint32_t, uint32_t) override;
    size_t read(std::vector<uint8_t>&, uint32_t) override;
    size_t write(const std::vector<uint8_t>&) override;
    void flushBuffer() override;

  private:
    const int32_t fd;

    void configure(speed_t);
};

class usb : public serial
{
  public:
    explicit usb(const std::string&, speed_t);
    explicit usb(const std::string&, speed_t, bool);
    ~usb();

    size_t read(std::vector<uint8_t>&, uint32_t, uint32_t) override;
    size_t read(std::vector<uint8_t>&, uint32_t) override;
    size_t write(const std::vector<uint8_t>&) override;
    void flushBuffer() override;

  private:
    const int32_t fd;
    const bool debug;

    void disableFlowControl();
    uint32_t bytesInInput();
    void configure(speed_t);
    void showserialtraces(std::string_view, const std::vector<uint8_t>&);
};
