/**
  ******************************************************************************
  * @file    USBProtocol.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-21
  * @brief   Handles the USB protocol (application specific)
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "USBProtocol.hpp"
//------------------------------------------------------------------------------
#include "usb_com.hpp"
#include "usb_tags.hpp"
#include "misc.h"
#include <cstring>
#include <algorithm>
#include <memory>
#include <array>
#include <numeric>
#include "Input.hpp"
#include "vsense.hpp"
#include "bsp/TimedTasks.hpp"
//------------------------------------------------------------------------------

using std::array;

static constexpr size_t MAX_LEN  = 1024;
static const uint16_t   StartMsg = 0xFFFF;

static const uint8_t MotorIDs[] =
{
    HerkulexMan::ID_R_HIP_YAW,       HerkulexMan::ID_L_HIP_YAW,
    HerkulexMan::ID_R_HIP_ROLL,      HerkulexMan::ID_L_HIP_ROLL,
    HerkulexMan::ID_R_HIP_PITCH,     HerkulexMan::ID_L_HIP_PITCH,
    HerkulexMan::ID_R_KNEE,          HerkulexMan::ID_L_KNEE,
    HerkulexMan::ID_R_ANKLE_PITCH,   HerkulexMan::ID_L_ANKLE_PITCH,
    HerkulexMan::ID_R_ANKLE_ROLL,    HerkulexMan::ID_L_ANKLE_ROLL,
    HerkulexMan::ID_HEAD_PAN,
    HerkulexMan::ID_HEAD_TILT
};
//------------------------------------------------------------------------------
static void wait(unsigned us)
{
    bsp::CPUclockdiff cpu;

    while (cpu.cpudiff() < (cpu.cycle2us * us)) {}
}
//------------------------------------------------------------------------------
static ssize_t usb_writewait(const void *p, size_t len)
{
    size_t wr;
    ssize_t res = 0;

    unsigned full_cnt = 0;

    auto ptr = static_cast<const uint8_t *>(p);
    do
    {
        auto ret = usb::write(ptr, len);
        if (ret < 0) return ret;
        if (ret == 0)
        {
            if (full_cnt++ > 10)
                return -1;
            wait(10);
        }
        // ret is positive, so it should fit in a size_t
        wr = static_cast<size_t>(ret);
        ptr += wr;
        res += wr;
    } while (len -= wr);
    return res;
}
//------------------------------------------------------------------------------
class USBProtocol::DataBuffer : public  Herkulex::MsgHandler,
                                private FIFOalloc<512>
{
private:
    union
    {
        // This member is for alignment purposes.
        uint64_t dummy;

        struct {
            uint8_t  dataIn [MAX_LEN];
            uint8_t  dataOut[MAX_LEN];
        };
    };

    size_t  idx;
    size_t  len;
    uint8_t checkSum;

    Input sw[3] = { inputMan.input(button::b1),
                    inputMan.input(button::b2),
                    inputMan.input(button::b3)
                    };

    USBProtocol* prot;

    enum { other_reads_reload = 50-1 };
    unsigned timer_other_reads= other_reads_reload;
    bool do_other_reads = false;

    array<usb_sub_message<usb_tag::MO>, 256> statusDatabase;

    unsigned numPackets() const volatile { return pushcnt - popdcnt; }
    unsigned pushcnt = 0;
    unsigned popdcnt = 0;

    void notify() { ++pushcnt; }
    void ack()    { ++popdcnt; }

public:
    void updOtherReads()
    {
        do_other_reads = !timer_other_reads;
        if (do_other_reads)
        {
            timer_other_reads = other_reads_reload;
        }
        else
        {
            --timer_other_reads;
        }
    }

    void Reset()
    {
        checkSum = 0;
        idx = 0;
        len = 0;
    }

    DataBuffer(USBProtocol* prot) : prot{prot}
    {
        Reset();
        std::fill(statusDatabase.begin(), statusDatabase.end(),
              usb_sub_message<usb_tag::MO>{0xFF});

        for (auto &s : sw)
        {
            s.configure(event_type::toggled);
        }
    }

    ~DataBuffer(){}

    class usb_out_iterator : public std::iterator<
        std::output_iterator_tag,
        void,
        std::ptrdiff_t,
        std::uint8_t *,
        std::uint8_t &
        >
    {
        DataBuffer &db;
        size_t   len;
        uint8_t *databuf;
        enum { max_len = MAX_LEN - sizeof (message_header_base) - 1};
    public:
        usb_out_iterator(DataBuffer *db) : db(*db) { reset(); }

        ~usb_out_iterator() {  db.sendUSB(len); }

        usb_out_iterator &operator++()       { return *this; }
        usb_out_iterator &operator++(int)    { return *this; }
        usb_out_iterator &operator*()        { return *this; }

        void reset()
        {
            databuf = db.datatoUSB();
            len     = 0;
        }

        template <typename T>
         usb_out_iterator &operator=(const T &t)
        {
            this->push(reinterpret_cast<const std::uint8_t*>(&t), sizeof(t));
            return *this;
        }

        void push(const uint8_t *datastart, size_t datalen)
        {
            send_if_full(datalen);
            std::memcpy(databuf, datastart, datalen);
            push(datalen);
        }

        void push(size_t datalen)
        {
            len     += datalen;
            databuf += datalen;
        }

        uint8_t *storage(size_t datalen)
        {
            if (datalen > max_len) return nullptr;

            send_if_full(datalen);

            return databuf;
        }
    private:
        void send_if_full(size_t datalen)
        {
            if ((len + datalen) > max_len )
            {
                db.sendUSB(len);
                reset();
            }
        }
    };

    void FillBuffer(uint8_t ch)
    {
        using std::memcmp;

        // store first
        dataIn[idx] = ch;

        if (idx >= 2) checkSum += ch;

        switch (idx++)
        {
        case 0:
        case 2:
            // Do nothing.
            break;
        case 1:
            if (memcmp(dataIn, &StartMsg, sizeof StartMsg))
            {
                // Error header!
                Reset();
            }
            break;
        case 3:
            len = reinterpret_cast<uint16_t &>(dataIn[2]);

            if (len > MAX_LEN)
            {
                // Error header! (length)
                Reset();
            }
            len += 3; // include header size + chksum
            break;
        default:
            if (idx == len)
            {
                // Cancel last sum, then substract packet checksum
                checkSum -= 2*ch;
                if (checkSum != 0)
                {
                    // Error checksum!
                }
                else
                {
                    const auto   ptr = dataIn + sizeof(message_header_base);
                    const size_t l   = len - (ptr - dataIn) - 1;
                    prot->decode(ptr, l);
                }
                Reset();
            }
            break;
        }
        //
    }

    ssize_t sendUSB(size_t s)
    {
        if (s >= (MAX_LEN - sizeof (message_header_base) - 1)) return 0;
        if (s == 0) return 0;

        auto &msg = reinterpret_cast<message_header_base&>(dataOut);

        uint8_t *const headerAddr  = dataOut;
        uint8_t *const sizeAddr    = msg.size.bytes;
        uint8_t *const chksumAddr  = headerAddr + s + sizeof msg;

        // Total size is s plus header and chksum
        const size_t totalmsglen = chksumAddr - headerAddr + 1;

        msg.tag  = StartMsg;
        msg.size = s + sizeof msg.size;

        *chksumAddr = std::accumulate(sizeAddr, chksumAddr, 0u);

        return usb_writewait(dataOut, totalmsglen);
    }

    void sendMR()
    {
        if (numPackets())
        {
            auto it = usb_out_iterator(this);
            //
            while (numPackets())
            {
                auto reg            = access();
                size_t fromlen      = reg.len;
                const uint8_t *from = reg.data;
                const auto advance  = fromlen + sizeof (message_header_base);

                auto p = it.storage(advance);

                auto &msg = *new (p) usb_message<usb_tag::MR> {};
                msg.size(fromlen);

                std::memcpy(&msg.data.id, from, fromlen);

                it.push(advance);
                pop();
                ack();
            }
        }
    }
    uint8_t *datatoUSB() { return dataOut + sizeof (message_header_base); }

    void sendMotorReads()
    {
        const auto h = prot->herkman->herkulex();

        //wait(200);
        for (auto id : MotorIDs)
        {
            h->send_ram_read(id, RAM_CALIBRATED_POSITION, 2);
            //wait(200);
            /*
            if (send_other_reads)
            {
                h->send_ram_read(id, RAM_VOLTAGE, 2);
                wait(300);
                h->send_ram_read(id, RAM_PWM, 2);
                wait(300);
            }
            */
        }
    }

    void sendDataupd()
    {

        auto it = usb_out_iterator(this);

        enum {  insz = usb_sub_message<usb_tag::MO>::inner_min_size };

        const unsigned leapmin = insz + sizeof (message_header_base);
        //const unsigned leapmax = sizeof(usb_message<usb_tag::MO>);

        const unsigned leap = leapmin;

        for (auto id : MotorIDs)
        {
            auto &dataentry = statusDatabase[id];

            // Proceed only if we have received data.
            if (dataentry.id == 0xFF)
                continue;

            // Set acknowledge
            dataentry.id = 0xFF;

            auto p = it.storage(leap);
            auto &msg = *new (p) usb_message<usb_tag::MO>;

            msg.size(insz);


            msg.data.id = id;
            msg.data.pos = dataentry.pos;
            msg.data.dt  = 0;
            msg.data.status[0] = dataentry.status[0];
            msg.data.status[1] = dataentry.status[1];

            it.push(leap);
        }

        for (auto &b : sw)
        {
            if (b.read())
            {
                usb_message<usb_tag::BU> msg {};
                msg.data.id = &b - sw;
                msg.data.val= b.isDown();
                *it++ = msg;
            }
        }

        if (do_other_reads)
        {
            const float volt = bsp::vsense.getVoltage();
            usb_message<usb_tag::PO> msg {};
            msg.data.val = 16.f * volt;
            *it++ = msg;
        }
    }

    void dotimedTasks(unsigned timer)
    {
        switch (timer)
        {
        case 10:
            sendMotorReads();
            break;
        case 1:
            sendDataupd();
            break;
        default:
            // Do nothing
            break;
        }
    }

    //--MsgHandler--------------------------------------------------------------
    // Errors are not handled
    // void err_header(msgPacket *) override {}
    // void err_checksum(msgPacket *) override {}

    void unknownPacket(msgPacket *m) override
    {
        return sendBack(m);
    }

    void stat(msgStat *m) override
    {
        return sendBack(m);
    }
    void eep_read(msgEEP_read *m) override
    {
        return sendBack(m);
    }
    void ram_read(msgRAM_read *m) override
    {
        const uint8_t &id   = m->buffer[msgPacket::idx::motor_id];
        const uint8_t &addr = m->buffer[msgPacket::idx::data + 0];
        const uint8_t &len  = m->buffer[msgPacket::idx::data + 1];

        const uint8_t *data = m->buffer + msgPacket::idx::data + 2;

        // If this id is not known to us, or the length is not 2,
        // we send this message to PC.
        if (std::find(std::begin(MotorIDs), std::end(MotorIDs), id)
             == std::end(MotorIDs) or len != 2)
            return sendBack(m);
        // implicit else

        switch (addr)
        {
        case RAM_CALIBRATED_POSITION:
            statusDatabase[id].pos = reinterpret_cast<const int16le &>(*data)
                                   & 0x3FF;
            break;
        case RAM_VOLTAGE:
            statusDatabase[id].voltage     = data[0];
            statusDatabase[id].temperature = data[1];
            break;
        case RAM_PWM:
            statusDatabase[id].PWM = reinterpret_cast<const uint16le &>(*data);
            break;
        default:
            // Not a known address? Send back!
            return sendBack(m);
        }

        // Update id, that will certify the data has been received
        statusDatabase[id].id = id;

        // Because we always send reads with a length of 2, we can be confident
        // that the status is there.
        statusDatabase[id].status[0] = data[2];
        statusDatabase[id].status[1] = data[3];
    }

    void sendBack(msgPacket *m)
    {
        // Count the len of data + id and cmd
        unsigned len = m->buffer[msgPacket::idx::packet_size] - MIN_PACKET_SIZE
                     + 2;

        fifo_ptr f{*this, len};

        if (!f)
        {
            // Oooops! Not enough memory in buffer! We unfortunately have to
            // drop it....
            return;
        }
        uint8_t *out = f.get();

        --len, *out++ = m->buffer[msgPacket::idx::motor_id];
        --len, *out++ = m->buffer[msgPacket::idx::command];

        const uint8_t *const in = m->buffer + msgPacket::idx::data;
        std::memcpy(out, in, len);

        f.release();
        notify();
    }
};
//------------------------------------------------------------------------------
class i_jog_buffer
{
public:
    i_jog_buffer()
    {
        Reset();
    }

    void Reset()
    {
        len = 0;
    }

    void insert(const i_jog_part &ij)
    {
        const size_t id  = ij.id;

        bufjog[id] = ij;

        const size_t newidx =  std::find(idxbuf, idxbuf+len, id) - idxbuf;

        // Either it was found (and we have its index) or it was not and newidx
        // is one greater than len; check if we're not outside the array.
        if (newidx < numel)
        {
            idxbuf[newidx] = id;
            // if it's a new index, increase len.
            len = std::max(newidx+1, len);
        }
    }

    i_jog_part &operator[](size_t i)
    {
        return bufjog[idxbuf[i]];
    }

    size_t length() const { return len; }
private:
    enum { numel = 256 };
    i_jog_part bufjog[numel];
    uint8_t    idxbuf[numel];
    size_t     len;
};
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
USBProtocol::USBProtocol(HerkulexMan *herkman)
 : herkman(herkman), dataBuff{new DataBuffer(this)}
{
    usb::init();

    herkman->herkulex()->handler(dataBuff.get());
}
//------------------------------------------------------------------------------
USBProtocol::~USBProtocol()
{
    herkman->herkulex()->resetHandler();
}
//------------------------------------------------------------------------------
void USBProtocol::loop()
{
    Input inputs[] =
    {
        inputMan.input(button::b1),
        inputMan.input(button::b2),
        inputMan.input(button::b3)
    };

    for (auto &i : inputs)
    {
        i.configure(event_type::pressed);
    }

    for (;;)
    {
        uint8_t minibuf[8];

        const auto readlen = usb::read(minibuf, sizeof minibuf);

        if (readlen < 0)
        {
            // Error! Reset buffer
            dataBuff->Reset();
        }
        else
        {
            for (auto p = minibuf; p < minibuf + readlen; ++p)
            {
                dataBuff->FillBuffer(*p);
            }

            dataBuff->sendMR();
        }

        const auto old_timer = timer;
        // Get time difference, normally it is 0 or 1
        unsigned delta_t = usb::getSOFCount() - sof_counter;

        sof_counter += delta_t;
        timer       -= delta_t;


        if (timer <= 0)
        {
            // It's a while loop, but normally, 0 or 1 iterations are done
            // inside
            while (timer <= 0)
                timer += reload10ms;

            dataBuff->updOtherReads();
        }

        if (timer != old_timer)
            dataBuff->dotimedTasks(timer);

        unsigned buttonsdown   = 0;
        bool     anypressed    = false;

        for (auto &i : inputs)
        {
            bool pressed = i.read();
            anypressed   = anypressed or pressed;
            buttonsdown += i.isDown();
        }


        if (anypressed and buttonsdown > 1)
        {
            herkman->herkulex()->setTorque(BROADCAST_ID, TORQUE_FREE);
        }


    }
}
//------------------------------------------------------------------------------
void USBProtocol::decode(uint8_t *data, uint32_t n)
{
    const auto data_end = data + n;

    i_jog_buffer buff{};

    do
    {
        auto &msg = reinterpret_cast<message_header_base&>(*data);

        switch (msg.tag)
        {
        case usb_tag_traits<usb_tag::MO>::tag():
            {
                auto constexpr utag = usb_tag::MO;

                const auto &decoded = reinterpret_cast<usb_message<utag>&>(msg);

                if (decoded.header.size < decoded.data.inner_min_size)
                {
                    // Ooops, too small message, skip...
                }
                else
                {
                    i_jog_part tmp{};
                    tmp.mode = POS_MODE;
                    tmp.pos = decoded.data.pos;
                    tmp.id  = decoded.data.id;
                    tmp.time= decoded.data.dt;
                    buff.insert(tmp);
                }
            }
            break;
        case usb_tag_traits<usb_tag::MR>::tag():
            {
                auto constexpr utag = usb_tag::MR;

                const auto &decoded = reinterpret_cast<usb_message<utag>&>(msg);

                if (decoded.header.size < decoded.min_size
                                                or decoded.header.size > 250)
                {
                    // Oops, too small/big message... skip it
                }
                else
                {
                    const auto h = herkman->herkulex();

                    const auto packetsize = decoded.header.size
                                            - sizeof (decoded.header.size)
                                            - sizeof (usb_sub_message<utag>);

                    // We use message no_cmd, which is kind of a custom message
                    auto p = h->newMsg<servocmd::no_cmd>
                                (decoded.data.id, packetsize);

                    p->buffer[msgPacket::idx::command] = decoded.data.cmd;

                    auto from = decoded.data.data;
                    for (auto &v : *p)
                    {
                        v = *from++;
                    }
                    h->sendPacket(p);
                }
            }
            break;
        case usb_tag_traits<usb_tag::TO>::tag():
            {
                auto constexpr utag = usb_tag::TO;

                const auto &decoded = reinterpret_cast<usb_message<utag>&>(msg);
                uint32_t const idx = decoded.data.val;

                if (decoded.header.size < decoded.min_size)
                {
                    // Oops, too small message... skip it
                }
                else
                {
                    herkman->herkulex()->setTorque(decoded.data.id,
                                            idx ? TORQUE_ON : TORQUE_FREE);
                }
            }
            break;
        default:
            // Oops, unsupported message, just skip it.
            break;
        }
        data += (msg.size + sizeof msg.tag);
    } while (data < data_end);

    // Transmit i_jog command, if there was some.
    if (buff.length())
    {
        size_t len = buff.length();
        constexpr size_t maxxfer = msgtraits<servocmd::i_jog>::max_elems;

        auto h = herkman->herkulex();

        do
        {
            // Never xmit more than the message can xmit
            const size_t thisxferlen = std::min(maxxfer, len);

            auto hmsg = h->newMsg<servocmd::i_jog>(BROADCAST_ID, thisxferlen);
            int i = 0;
            for (auto &v : *hmsg)
            {
                v = buff[i++];
            }
            h->sendPacket(hmsg);
            len -= thisxferlen;
        } while (len);
    }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

