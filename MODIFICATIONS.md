# Modifications

Changes to `supercamera_poc.cpp` to support a second hardware variant of the
`2ce3:3828` ‘Geek szitman supercamera’, found on a device reporting firmware
`bcdDevice 1.11`. The original two-interface (firmware 1.00) code path is unchanged.

Original project: [hbens/geek-szitman-supercamera](https://github.com/hbens/geek-szitman-supercamera).

Tested hardware: [this product on Amazon](https://a.co/d/088q74cF).

## The problem

On the new device the program failed with:

```
fatal: usb_claim_interface B error (-5) Entity not found
```

`lsusb -v` showed that this device has a different USB layout from the one in `descriptors/`:

| | Original (fw 1.00) | New variant (fw 1.11) |
|---|---|---|
| Interfaces | 2 | 1 |
| Interface 0 | EP2 bulk IN/OUT | EP2 bulk IN/OUT |
| Interface 1 alt 1 | EP1 bulk IN/OUT (video) | does not exist |
| Video format | JPEG in "UPP" packets, 640x480 | raw YUYV, 640x480 |

## New variant protocol

The start and stop requests come from
[KlumpRasmus/supercamera-yuyv-linux](https://github.com/KlumpRasmus/supercamera-yuyv-linux)
(which credits [Bognabon/Endoscope_Viewer](https://github.com/Bognabon/Endoscope_Viewer)).
That driver describes 320x240 frames. The frame format below comes from analysing a raw
capture of the fw 1.11 device instead.

- **Start:** clear halt on EP2 IN and OUT, then send these control transfers on EP0:
  - Get info: `bmRequestType 0xA0, bRequest 0x00, wValue 0x0005, wIndex 0`, 512 bytes IN.
    The result is only informative.
  - Camera up: `bmRequestType 0x20, bRequest 0x01, wValue 0x0005, wIndex 0`, 64 zero bytes
    OUT, then wait 200 ms.
- **Stop:** `bmRequestType 0x20, bRequest 0x02, wValue 0x0005, wIndex 0`, no data.
- **No bulk OUT writes.** Writing to EP2 OUT times out.
- **Stream:** a continuous byte stream on bulk EP2 IN. Each frame is:
  - a 511-byte header starting with `dd cc 01 00 60 09 00 00 00`. The header was
    byte-for-byte identical across frames in the capture.
  - followed by 640x480 YUYV pixel data, 1280 bytes per row.
- **Lost data:** the device does not wait for the computer. If no IN transfer is pending,
  data is lost and frames arrive short by some rows. Several transfers must stay queued.

## Code changes

- **Layout detection:** `has_interface()` reads the active configuration descriptor. If
  interface 1 is missing, the program switches to the single-interface "YUYV mode" instead
  of failing.
- **YUYV mode start and stop:**
  - `yuyv_start()` sends the control transfers above.
  - `yuyv_stop()`, called from the destructor, cancels the pending transfers and sends the
    camera-down request.
  - The original EP1/EP2 bulk "magic words" are skipped in this mode.
- **Async reads:** 8 x 64 KB asynchronous bulk transfers stay queued at all times
  (`yuyv_transfer_cb` resubmits each one when it completes), so the device always has
  somewhere to send data.
- **Frame splitting:** `read_yuyv_frame()` buffers the stream and cuts frames between
  consecutive `dd cc 01 00` headers.
  - The 511-byte header is removed.
  - A short frame is padded with black rows to 640x480.
  - The buffer is resynced if no header is found.
- **Display:** a new `yuyv()` thread function converts each frame with
  `cv::cvtColor(..., COLOR_YUV2BGR_YUYV)` and re-encodes it as JPEG. The existing GUI and
  picture-saving code then works unchanged. `upp()` dispatches to it in YUYV mode.
- **Debugging:** setting `SUPERCAMERA_DUMP=<file>` writes the raw EP2 stream to a file for
  offline analysis.
- **Includes:** added `<algorithm>`, `<cstdlib>`, `<opencv2/imgcodecs.hpp>` and
  `<opencv2/imgproc.hpp>`.
- **Quieter output:** the per-frame `PIC i:... size:...` debug line in `pic_callback()` is
  commented out.

## Known limitations

- The endoscope button is not handled in YUYV mode. Its encoding in this stream, if any,
  is unknown.
- Only one fw 1.11 device has been tested.
- It is not known whether the header bytes after `dd cc 01 00` carry meaningful fields.
