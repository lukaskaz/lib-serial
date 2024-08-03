#pragma once

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

    size_t read(std::vector<uint8_t>&, ssize_t);
    size_t read(std::vector<uint8_t>&, ssize_t, uint32_t);
    size_t write(const std::vector<uint8_t>&);

    std::string getdevice() const;
    std::string getbaud() const;

  protected:
    std::string device;
    std::string baudname;

    void showserialtraces(std::string_view, const std::vector<uint8_t>&,
                          debug_t) const;
    std::string getbaudname(speed_t) const;
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
