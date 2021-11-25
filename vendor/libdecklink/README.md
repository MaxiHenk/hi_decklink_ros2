# LibDeckLink

libdecklink is a wrapper around the DeckLink SDK. It currently only supports Linux since that is what I have on my development machine but does not do anything platform-specific. It should be pretty simple to port. 

This library tries to provide a wrapper around a subset of the DeckLink SDK to make it simpler to use. The main goal is to create a succinct interface to the hardware without forgoing all the checks required when calling into the DeckLink device driver. 

When using the library you can be confident that all the possible return codes are checked and in case of errors a descriptive exception will be thrown bubbling up all the relevant information. It also wraps all the raw enumeration in `enum class` to enforce type safety and reduce errors. 

## Installation Instructions

To download and build the library you should run the following commands: 

    ~ git clone --recursive https://gitlab.com/Polimi-dVRK/decklink/libdecklink.git

    ~ cd libdecklink
    ~ mkdir cmake-build
    ~ cd cmake-build
    ~ cmake ..
    ~ make

To use the library in your application clone it as a git submodule into your code and use the `CMake` command `add_subdirectory` to make it available as a target to `CMake`.

## Example usage

The entry point to the library is the `DeckLink::Device` class. You can get a vector containing all the detected devices with: 

    const auto devices = DeckLink::Device::All();

or you can get a single device with:

    boost::optional<DeckLink::Device> result = 
        DeckLink::Device::Get("DeckLink Duo (1)");
    if (!result) {
        std::cerr << "Device not found\n";
    } else {
        DeckLink::Device device = std::move(*result);
    }

To read images from the device we need to create a callback function to call when new images arrive: 

    void on_new_image(DeckLink::VideoInputFrame frame) {
        // Do something
    }

and attach it to a `CaptureCallback`

    DeckLink::CaptureCallback cb(on_new_image);

which we finally attach to the input interface of the device:

    device.input()
        .set_callback(cb)
        .enable()
        .start();

When we are done we can stop the capture with: 

    device.input().stop();

For more information check out the doxygen documentation or the tools in `src/`. In general the library has a similar interface to the _official_ SDK so the samples there mught provide an idea of how things work here. 

## API Coverage

This library only wraps a subset of the official DeckLink SDK. It has however been developped with the objective of providing easy access to the underlying hardware when needed. Most classes expose the underlying `IDeckLink*` pointer through their `get_raw_device()` method.

| API Section                                       | Status  |
| ------------------------------------------------- | ------- |
| Attributes Interface (IIDeckLinkAttributes)       | Yes     | 
| Configuration Interface (IIDeckLinkConfiguration) | Yes     | 
| Status Interface (IIDeckLinkStatus)               | Yes     | 
| Notification Interface (IIDeckLinkNotification)   | Yes     | 
| Input Interface (IIDeckLinkInput)                 | Yes     | 
| Output Interface (IIDeckLinkOutput)               | Yes     |
| Keyer Interface (IIDDeckLinkKeyer)                | Yes     |

Interfaces not indicated here are not supported. If you need them open an issue. 

## Samples

You can find usage samples for the API in the `src/` directory. Each sample has complete `--help` output.
