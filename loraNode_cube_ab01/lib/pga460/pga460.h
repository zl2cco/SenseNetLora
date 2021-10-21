#ifndef pga460_h
#define pga460_h
#include "Arduino.h"
#include "softSerial.h"


struct obj_t {
    uint8_t   node_id;
    double    distance_m;
    double    vbat;
    double    temp;
    uint32_t  crc32;
};

class PGA460
{
  public:
    PGA460(int tx_pin, int rx_pin, int en_pin, int tst_pin);
    void begin();
    void end();
    int  get_status();
    void deep_sleep();
    double get_distance(obj_t *obj);
    bool getNOBALdata(uint8_t *data, uint8_t *diag);
    void set_tv_gain(uint8_t dB);
  private:
    int _en_pin;
    int _tst_pin;
    softSerial* _serial_port;
    int status;

    uint8_t calc_checksum(uint8_t* buf, int len);
    void writereg(uint8_t reg, uint8_t data);
    bool readreg(uint8_t regaddr, uint8_t* diag, uint8_t* regdata);

    void set_thresholds(byte thr);
    bool getUSmeasurement(byte preset, uint16_t *tof, uint8_t *width, uint8_t* peak_amp, uint8_t *diag);

    void write_default_values();


};




#endif