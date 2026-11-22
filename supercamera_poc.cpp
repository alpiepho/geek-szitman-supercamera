/*
 * Proof of concept for the 'Geek szitman supercamera' endoscope
 *
 * This endoscope uses the 'com.useeplus.protocol' protocol.
 * Only hardware revision 1.00 was tested.
 *
 * SPDX-License-Identifier: CC0-1.0
 * by hbens
 *
 * Thanks to: doctormo, jmz3, RGBA-CRT
 */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <version>

#ifndef __cpp_lib_format
#include <ctime>
#endif

#include <libusb-1.0/libusb.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wdeprecated-anon-enum-enum-conversion"
#endif
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#pragma GCC diagnostic pop

static constexpr int VERBOSE = 0;

#define KRST "\e[0m"
#define KRED "\e[0;31m"
#define KGRN "\e[0;32m"
#define KYLW "\e[0;33m"
#define KBLU "\e[0;34m"
#define KMAJ "\e[0;35m"
#define KCYN "\e[0;36m"

using byteVector = std::vector<uint8_t>;
using vid_pid_t = std::pair<uint16_t, uint16_t>;

class UsbSupercamera {
    static constexpr vid_pid_t USB_VENDOR_PRODUCT_ID_LIST[] =
        {{0x2ce3, 0x3828}, {0x0329, 0x2022}};
    static constexpr int INTERFACE_A_NUMBER = 0;
    static constexpr int INTERFACE_B_NUMBER = 1;
    static constexpr int INTERFACE_B_ALTERNATE_SETTING = 1;
    static constexpr unsigned char ENDPOINT_1 = 1;
    static constexpr unsigned char ENDPOINT_2 = 2;
    static constexpr unsigned int USB_TIMEOUT = 1000; /* in ms */

    libusb_context *ctx;
    libusb_device_handle *handle;
    /*
     * Firmware 1.00 exposes 2 interfaces: control on interface 0 (EP2) and a JPEG "UPP" video
     * stream on interface 1 alt 1 (EP1).
     * Some devices with the same VID:PID (seen with firmware 1.11) expose only interface 0 and
     * use a different protocol: started/stopped with EP0 control transfers (from
     * github.com/KlumpRasmus/supercamera-yuyv-linux and github.com/Bognabon/Endoscope_Viewer),
     * then a continuous byte stream on bulk EP2 IN.
     * Observed on a fw 1.11 device: each frame is a 511-byte header starting with dd cc 01 00,
     * followed by 640x480 YUYV rows. The device does not wait for the host: if no IN transfer
     * is pending, data is dropped (frames arrive short by some rows). Several async transfers
     * are therefore kept in flight.
     */
    bool yuyv = false;
    bool yuyv_stopping = false;
    bool yuyv_device_gone = false;
    int yuyv_in_flight = 0;
    std::vector<libusb_transfer *> yuyv_transfers;
    byteVector yuyv_stream; /* bytes received but not yet returned as a frame */
    std::ofstream yuyv_dump; /* raw stream dump, enabled with env SUPERCAMERA_DUMP=<file> */

public:
    static constexpr int YUYV_WIDTH = 640;
    static constexpr int YUYV_HEIGHT = 480;
    static constexpr size_t YUYV_FRAME_SIZE = YUYV_WIDTH * YUYV_HEIGHT * 2;
private:
    static constexpr size_t YUYV_HEADER_SIZE = 511;
    static constexpr uint8_t YUYV_MAGIC[] = {0xdd, 0xcc, 0x01, 0x00};
    static constexpr int YUYV_NUM_TRANSFERS = 8;
    static constexpr int YUYV_TRANSFER_SIZE = 64 * 1024;

    static void LIBUSB_CALL yuyv_transfer_cb(libusb_transfer *t) {
        auto *self = static_cast<UsbSupercamera *>(t->user_data);
        self->yuyv_in_flight--;
        if (t->status == LIBUSB_TRANSFER_COMPLETED) {
            if (self->yuyv_dump.is_open()) {
                self->yuyv_dump.write(reinterpret_cast<const char *>(t->buffer), t->actual_length);
            }
            self->yuyv_stream.insert(self->yuyv_stream.end(), t->buffer, t->buffer + t->actual_length);
            if (VERBOSE > 1) {
                std::cerr << KGRN "   IN " << t->actual_length << KRST << std::endl;
            }
        } else if (t->status == LIBUSB_TRANSFER_NO_DEVICE) {
            self->yuyv_device_gone = true;
        } else if (t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_TIMED_OUT) {
            std::cerr << KRED "yuyv transfer status " << t->status << KRST << std::endl;
        }
        if (!self->yuyv_stopping && !self->yuyv_device_gone) {
            if (libusb_submit_transfer(t) == 0) {
                self->yuyv_in_flight++;
            }
        }
    }

    int yuyv_start() {
        int ret;
        libusb_clear_halt(handle, LIBUSB_ENDPOINT_IN | ENDPOINT_2);
        libusb_clear_halt(handle, LIBUSB_ENDPOINT_OUT | ENDPOINT_2);

        /* getinfo: vendor IN request, result only informative */
        unsigned char info[512];
        ret = libusb_control_transfer(handle, 0xA0, 0x00, 0x0005, 0x0000, info, sizeof(info), USB_TIMEOUT);
        if (ret < 0) {
            std::cerr << "warning: yuyv getinfo error (" << ret << ") " << libusb_strerror(ret) << std::endl;
        } else if (VERBOSE > 0) {
            std::cerr << KGRN "   getinfo " << ret << " bytes" KRST << std::endl;
        }

        /* camera up: class OUT request with 64 zero bytes */
        unsigned char up[64] = {};
        ret = libusb_control_transfer(handle, 0x20, 0x01, 0x0005, 0x0000, up, sizeof(up), USB_TIMEOUT);
        if (ret < 0) {
            std::cerr << "fatal: yuyv camera up error (" << ret << ") " << libusb_strerror(ret) << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        yuyv_stream.clear();
        if (const char *dump_path = std::getenv("SUPERCAMERA_DUMP")) {
            yuyv_dump.open(dump_path, std::ios::binary);
            std::cerr << KYLW "info: dumping raw stream to " << dump_path << KRST << std::endl;
        }

        for (int i = 0; i < YUYV_NUM_TRANSFERS; i++) {
            libusb_transfer *t = libusb_alloc_transfer(0);
            auto *buf = new unsigned char[YUYV_TRANSFER_SIZE];
            libusb_fill_bulk_transfer(t, handle, LIBUSB_ENDPOINT_IN | ENDPOINT_2, buf, YUYV_TRANSFER_SIZE,
                                      yuyv_transfer_cb, this, USB_TIMEOUT);
            yuyv_transfers.push_back(t);
            ret = libusb_submit_transfer(t);
            if (ret < 0) {
                std::cerr << "fatal: yuyv submit transfer error (" << ret << ") " << libusb_strerror(ret) << std::endl;
                return 1;
            }
            yuyv_in_flight++;
        }
        return 0;
    }

    void yuyv_stop() {
        yuyv_stopping = true;
        for (auto *t : yuyv_transfers) {
            libusb_cancel_transfer(t);
        }
        while (yuyv_in_flight > 0) {
            timeval tv = {1, 0};
            if (libusb_handle_events_timeout(ctx, &tv) < 0) {
                break;
            }
        }
        if (yuyv_in_flight == 0) {
            for (auto *t : yuyv_transfers) {
                delete[] t->buffer;
                libusb_free_transfer(t);
            }
        }
        yuyv_transfers.clear();
        if (!yuyv_device_gone) {
            libusb_control_transfer(handle, 0x20, 0x02, 0x0005, 0x0000, nullptr, 0, USB_TIMEOUT);
        }
    }

    size_t yuyv_find_magic(size_t from) const {
        auto it = std::search(yuyv_stream.begin() + from, yuyv_stream.end(),
                              std::begin(YUYV_MAGIC), std::end(YUYV_MAGIC));
        return it - yuyv_stream.begin();
    }

    bool has_interface(int interface_number) {
        libusb_config_descriptor *config;
        int ret = libusb_get_active_config_descriptor(libusb_get_device(handle), &config);
        if (ret < 0) {
            std::cerr << "warning: libusb_get_active_config_descriptor error (" << ret << ") "
                      << libusb_strerror(ret) << std::endl;
            return true; /* assume the original layout */
        }
        bool found = false;
        for (int i = 0; i < config->bNumInterfaces; i++) {
            for (int a = 0; a < config->interface[i].num_altsetting; a++) {
                if (config->interface[i].altsetting[a].bInterfaceNumber == interface_number) {
                    found = true;
                }
            }
        }
        libusb_free_config_descriptor(config);
        return found;
    }

    int usb_read(unsigned char endpoint, byteVector &buf, size_t max_size, int debug) {
        int ret;
        int transferred;
        buf.resize(max_size);
        ret = libusb_bulk_transfer(handle, LIBUSB_ENDPOINT_IN | endpoint,
                                   buf.data(), buf.size(), &transferred, USB_TIMEOUT);
        if (ret != 0) {
            std::cerr << KRED "USB READ ERROR (" << ret << ") " << libusb_strerror(ret) << KRST << std::endl;
            buf.resize(0);
            return ret;
        }
        if (debug > 0) {
            std::ostringstream ss;
            ss << KGRN "   IN " << std::setw(5) << transferred;
            if (debug > 1) {
                ss << " << [";
                int max_print = std::min(10, transferred);
                if (debug > 2) {
                    max_print = transferred;
                }
                for (int i=0; i<max_print; i++) {
                    ss << " " << std::setfill('0') << std::setw(2) << std::hex << static_cast<unsigned>(buf[i]);
                }
                if (max_print < transferred) {
                    ss << " ...";
                }
                ss << " ]";
            }
            ss << KRST << std::endl;
            std::cerr << ss.str();
        }
        buf.resize(transferred);
        return 0;
    }

    int usb_write(unsigned char endpoint, byteVector buf, int debug) {
        int ret;
        int transferred;
        ret = libusb_bulk_transfer(handle, LIBUSB_ENDPOINT_OUT | endpoint,
                                   buf.data(), buf.size(), &transferred, USB_TIMEOUT);
        if (ret != 0) {
            std::cerr << KRED "USB WRITE ERROR (" << ret << ") " << libusb_strerror(ret) << KRST << std::endl;
            return -1;
        }
        if (debug > 0) {
            std::ostringstream ss;
            ss << KYLW "   OUT >> [";
            for (int i=0; i<transferred; i++) {
                ss << " " << std::setfill('0') << std::setw(2) << std::hex << static_cast<unsigned>(buf[i]);
            }
            ss << " ]" KRST << std::endl;
            std::cerr << ss.str();
        }
        return 0;
    }

    /* Derived from libusb `libusb_open_device_with_vid_pid_list` */
    libusb_device_handle *libusb_open_device_with_vid_pid_list(
        libusb_context *ctx, std::span<const vid_pid_t> vid_pid_list) {
        struct libusb_device **devs;
        struct libusb_device *found = nullptr;
        struct libusb_device *dev;
        struct libusb_device_handle *dev_handle = nullptr;

        if (libusb_get_device_list(ctx, &devs) < 0) {
            return nullptr;
        }

        size_t i = 0;
        while ((dev = devs[i++]) != nullptr) {
            struct libusb_device_descriptor desc;
            int ret = libusb_get_device_descriptor(dev, &desc);
            if (ret < 0) {
                goto out;
            }
            for (const auto &vid_pid : vid_pid_list)
            if (desc.idVendor == vid_pid.first && desc.idProduct == vid_pid.second) {
                found = dev;
                break;
            }
        }

        if (found) {
            int ret = libusb_open(found, &dev_handle);
            if (ret < 0) {
                dev_handle = NULL;
            }
        }

        out:
        libusb_free_device_list(devs, 1);
        return dev_handle;
    }

    int setup() {
        int ret;

        ret = libusb_init(&ctx);
        if (ret < 0) {
            std::cerr << "fatal: libusb_init fail (" << ret << ") " << libusb_strerror(ret) << std::endl;
            return 1;
        }

        handle = libusb_open_device_with_vid_pid_list(ctx, std::span(USB_VENDOR_PRODUCT_ID_LIST));
        if (!handle) {
            std::cerr << "fatal: usb device not found" << std::endl;
            return 1;
        }

        ret = libusb_claim_interface(handle, INTERFACE_A_NUMBER);
        if (ret < 0) {
            std::cerr << "fatal: usb_claim_interface A error (" << ret << ") " << libusb_strerror(ret) << std::endl;
            return 1;
        }

        if (!has_interface(INTERFACE_B_NUMBER)) {
            std::cerr << KYLW "info: no interface " << INTERFACE_B_NUMBER
                      << ", using single-interface YUYV 640x480 mode" KRST << std::endl;
            yuyv = true;
            return yuyv_start();
        }

        ret = libusb_claim_interface(handle, INTERFACE_B_NUMBER);
        if (ret < 0) {
            std::cerr << "fatal: usb_claim_interface B error (" << ret << ") " << libusb_strerror(ret) << std::endl;
            return 1;
        }

        ret = libusb_set_interface_alt_setting(handle, INTERFACE_B_NUMBER, INTERFACE_B_ALTERNATE_SETTING);
        if (ret < 0) {
            std::cerr << "fatal: libusb_set_interface_alt_setting B error (" << ret << ") "
                      << libusb_strerror(ret) << std::endl;
            return 1;
        }

        ret = libusb_clear_halt(handle, ENDPOINT_1);
        if (ret < 0) {
            std::cerr << "fatal: libusb_clear_halt EP1 error (" << ret << ") " << libusb_strerror(ret) << std::endl;
            return 1;
        }

        return 0;
    }

public:
    UsbSupercamera() {
        int ret = setup();
        if (ret != 0) {
            throw 1;
        }
        if (yuyv) {
            return;
        }
        /* Hey witch doctor, give us the magic words */
        byteVector ep2_buf = {0xFF, 0x55, 0xFF, 0x55, 0xEE, 0x10};
        usb_write(ENDPOINT_2, ep2_buf, VERBOSE);
        byteVector start_stream = {0xBB, 0xAA, 5, 0, 0};
        usb_write(ENDPOINT_1, start_stream, VERBOSE);
    }

    ~UsbSupercamera() {
        if (yuyv) {
            yuyv_stop();
        }
        libusb_close(handle);
        libusb_exit(ctx);
    }

    bool is_yuyv() const {
        return yuyv;
    }

    int read_frame(byteVector &read_buf) {
        return usb_read(ENDPOINT_1, read_buf, 0x400, VERBOSE);
    }

    /*
     * YUYV mode: on success, frame holds exactly YUYV_FRAME_SIZE bytes (padded with black
     * rows if the device sent fewer). Returns 1 when no complete frame is available yet.
     */
    int read_yuyv_frame(byteVector &frame) {
        /* A frame is complete when the next header has arrived */
        size_t start = yuyv_find_magic(0);
        if (start == yuyv_stream.size()) {
            /* no header: drop everything but a possible partial magic at the end */
            size_t keep = std::min(yuyv_stream.size(), sizeof(YUYV_MAGIC) - 1);
            yuyv_stream.erase(yuyv_stream.begin(), yuyv_stream.end() - keep);
        } else {
            if (start > 0) {
                yuyv_stream.erase(yuyv_stream.begin(), yuyv_stream.begin() + start);
            }
            if (yuyv_stream.size() > YUYV_HEADER_SIZE) {
                size_t next = yuyv_find_magic(YUYV_HEADER_SIZE);
                if (next < yuyv_stream.size()) {
                    size_t len = std::min(next - YUYV_HEADER_SIZE, YUYV_FRAME_SIZE);
                    if (VERBOSE > 0 && len != YUYV_FRAME_SIZE) {
                        std::cerr << KYLW "short frame: " << len / (YUYV_WIDTH * 2) << " rows" KRST << std::endl;
                    }
                    frame.assign(yuyv_stream.begin() + YUYV_HEADER_SIZE, yuyv_stream.begin() + YUYV_HEADER_SIZE + len);
                    frame.resize(len & ~size_t(3)); /* keep YUYV macro-pixel alignment */
                    while (frame.size() < YUYV_FRAME_SIZE) { /* black padding */
                        frame.push_back(0x10);
                        frame.push_back(0x80);
                    }
                    yuyv_stream.erase(yuyv_stream.begin(), yuyv_stream.begin() + next);
                    return 0;
                }
                if (yuyv_stream.size() > 4 * YUYV_FRAME_SIZE) { /* lost sync, start over */
                    yuyv_stream.clear();
                }
            }
        }

        if (yuyv_device_gone) {
            return LIBUSB_ERROR_NO_DEVICE;
        }
        timeval tv = {1, 0};
        int ret = libusb_handle_events_timeout(ctx, &tv);
        if (ret < 0) {
            return ret;
        }
        return 1;
    }
};

class UPPCamera {
    /* All packed struct fields are little-endian */
    static_assert(std::endian::native == std::endian::little);

    struct [[gnu::packed]] upp_usb_frame_t {
        uint16_t magic;
        uint8_t cid; /* camera id */
        uint16_t length; /* does not include the 5-bytes header length */
    };

    struct [[gnu::packed]] upp_cam_frame_t {
        uint8_t fid; /* frame id */
        uint8_t cam_num; /* camera number */
        /* misc flags in the next byte */
        unsigned char has_g:1;
        unsigned char button_press:1;
        unsigned char other:6;
        uint32_t g_sensor;
    };

    typedef void(*btn_callback_t)();
    typedef void(*pic_callback_t)(const byteVector &pic);
    pic_callback_t pic_callback;
    btn_callback_t btn_callback;

    static constexpr uint16_t UPP_USB_MAGIC = 0xBBAA;
    static constexpr uint8_t UPP_CAMID_7 = 7;
    static constexpr uint8_t UPP_CAMID_11 = 11;

    byteVector camera_buffer;
    upp_cam_frame_t cam_header = {};

public:
    UPPCamera(pic_callback_t pic_callback, btn_callback_t btn_callback) {
        this->pic_callback = pic_callback;
        this->btn_callback = btn_callback;
    }

    void handle_upp_frame(const byteVector &data) {
        /* Decode upp_usb_frame_t */
        size_t usb_header_len = sizeof(upp_usb_frame_t);
        if (data.size() < usb_header_len) {
            std::cerr << __func__ << " usb frame too small" << std::endl;
            return;
        }
        const upp_usb_frame_t *frame = (upp_usb_frame_t *) data.data();
        if (frame->magic != UPP_USB_MAGIC) {
            std::cerr << __func__ << " usb frame bad magic" << std::endl;
            return;
        }
        if ((frame->cid != UPP_CAMID_7) && (frame->cid != UPP_CAMID_11)) {
            std::cerr << __func__ << " unknown camera ID (got" << (int)frame->cid << ")" << std::endl;
            return;
        }
        if (usb_header_len + frame->length > data.size()) {
            /*
             * Used to be an equality check.
             * Allow extra bytes after the frame. With devices 0329:2022, corresponds to the
             *  beginning of the next frame, which will be retransmitted next read.
             */
            std::cerr << __func__ << " bad usb frame length (" << usb_header_len + frame->length
                      << ">" << data.size() <<")" << std::endl;
            return;
        }

        /* Decode upp_cam_frame_t */
        size_t cam_header_len = sizeof(upp_cam_frame_t);
        if (data.size() - usb_header_len < cam_header_len) {
            std::cerr << __func__ << " cam frame too small" << std::endl;
            return;
        }
        const upp_cam_frame_t *p_cam_header = (upp_cam_frame_t *) (data.data() + usb_header_len);

        if ((camera_buffer.size() > 0) && (cam_header.fid != p_cam_header->fid)) {
            pic_callback(camera_buffer);
            camera_buffer.resize(0);
        }

        if (camera_buffer.size() == 0) {
            cam_header = *p_cam_header;
            if (!(   (cam_header.cam_num < 2)
                  && (cam_header.has_g == 0)
                  && (cam_header.other == 0))) {
                std::cerr << __func__ << " bad first cam header" << std::endl;
                return;
            }
        } else {
            if (!(   (cam_header.fid == p_cam_header->fid)
                  && (cam_header.cam_num == p_cam_header->cam_num)
                  && (cam_header.has_g == p_cam_header->has_g)
                  && (cam_header.other == p_cam_header->other))) {
                std::cerr << __func__ << " bad continuation cam header" << std::endl;
                return;
            }
        }
        if (p_cam_header->button_press) {
            btn_callback();
        }

        auto cam_data_start = data.begin() + usb_header_len + cam_header_len;
        auto cam_data_end = data.begin() + usb_header_len + frame->length;
        camera_buffer.insert(camera_buffer.end(), cam_data_start, cam_data_end);
    }
};

static std::mutex gui_mtx; /* Protects latest_frame */
static byteVector latest_frame;
static std::atomic_uint32_t latest_frame_id;
static std::atomic_bool save_next_frame = false;
static std::atomic_bool exit_program = false;
static constexpr std::string_view pic_dir = "pics";

static void pic_callback(const byteVector &pic)
{
    static uint32_t i = 0;

    // std::cout << KCYN "PIC i:" << i << " size:" << pic.size() << KRST << std::endl;

    if (save_next_frame) {
        save_next_frame = false;
        std::ostringstream filename;
        auto tp = std::chrono::system_clock::now();

#ifdef __cpp_lib_format
        std::string date = std::format("{:%FT%T}", std::chrono::floor<std::chrono::seconds>(tp));
#else /* backup code for old compilers */
        std::time_t t = std::chrono::system_clock::to_time_t(tp);
        auto date = std::put_time(std::localtime(&t), "%FT%T");
#endif

        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count() % 1000;
        filename << pic_dir << "/frame_" << date
                 << "." << std::setfill('0') << std::setw(3) << millis << ".jpg";
        std::ofstream output(filename.str(), std::ios::binary);
        output.write(reinterpret_cast<const char *>(pic.data()), pic.size());
        std::cout << "Saved frame to " << filename.str() << std::endl;
    }

    {
        std::lock_guard lock(gui_mtx);
        latest_frame = pic;
        latest_frame_id = i;
    }

    i++;
}

static void button_callback() {
    std::cout << KMAJ "BUTTON PRESS" KRST << std::endl;
    save_next_frame = true;
}

static void gui(void) {
    constexpr const char *window_name = "Geek szitman supercamera - PoC";
    uint32_t frame_done = latest_frame_id;

    while (!exit_program) {
        int key = cv::waitKey(10);
        if (key == 'q' or key == '\e') {
            exit_program = true;
        }

        if (frame_done != latest_frame_id) {
            cv::Mat img;
            {
                std::lock_guard lock(gui_mtx);
                img = cv::imdecode(latest_frame, cv::IMREAD_COLOR);
                frame_done = latest_frame_id;
            }
            if (img.data != nullptr) {
                cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
                cv::imshow(window_name, img);
            }
        }
    }

    cv::destroyWindow(window_name);
}

/* Single-interface devices: raw YUYV frames, re-encoded as JPEG so the GUI/save path is shared */
static void yuyv(UsbSupercamera *usb_supercamera) {
    byteVector read_buf;
    byteVector jpeg;

    while (!exit_program) {
        int ret = usb_supercamera->read_yuyv_frame(read_buf);
        if (ret == 0) {
            cv::Mat yuyv_img(UsbSupercamera::YUYV_HEIGHT, UsbSupercamera::YUYV_WIDTH, CV_8UC2, read_buf.data());
            cv::Mat bgr;
            cv::cvtColor(yuyv_img, bgr, cv::COLOR_YUV2BGR_YUYV);
            cv::imencode(".jpg", bgr, jpeg);
            pic_callback(jpeg);
        } else if (ret == LIBUSB_ERROR_NO_DEVICE) {
            exit_program = true;
        }
    }
}

static void upp(UsbSupercamera *usb_supercamera) {
    if (usb_supercamera->is_yuyv()) {
        yuyv(usb_supercamera);
        return;
    }
    UPPCamera upp_camera(pic_callback, button_callback);
    byteVector read_buf;

    while (!exit_program) {
        int ret = usb_supercamera->read_frame(read_buf);
        if (ret == 0) {
            upp_camera.handle_upp_frame(read_buf);
        } else if (ret == LIBUSB_ERROR_NO_DEVICE) {
            exit_program = true;
        }
    }
}

int main(void)
{
    try {
        UsbSupercamera usb_supercamera;

        std::filesystem::create_directory(pic_dir);
        latest_frame_id = 0;

        std::thread upp_thread(upp, &usb_supercamera);
        gui();

        upp_thread.join();
        return 0;
    } catch (...) {
        return 1;
    }
}
