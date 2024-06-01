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

class usb : public serial
{
  public:
    explicit usb(const std::string&, speed_t);
    ~usb();

    size_t read(std::vector<uint8_t>&, uint32_t, uint32_t) override;
    size_t read(std::vector<uint8_t>&, uint32_t) override;
    size_t write(const std::vector<uint8_t>&) override;
    void flushBuffer() override;

  private:
    const int32_t fd;

    void disableFlowControl();
    void configureSerial(speed_t);
};
