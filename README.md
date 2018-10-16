# rc_visard OpenCV Example

Example showing how to use the `rc_genicam_api` with OpenCV
to receive images from rc_visard. 

## Build

Dependencies are `rc_genicam_api` and `OpenCV` (version 2 or 3).

### Linux

~~~bash
mkdir build && cd build
cmake ..
make
~~~

### Windows

The following instructions work with [cmake-gui](https://cmake.org/) and
[Microsoft Visual Studio-IDE](https://visualstudio.microsoft.com/vs/).

1. Run cmake-gui and set the source folder to 
   the repository's folder and the binaries folder to `build` in the 
   repository's folder
2. Click *Configure* 
3. In the pop-up, choose *Visual Studio*
4. Set `OpenCV_DIR` and `RC_GENICAM_API_DIR` appropriately
5. Set `CMAKE_INSTALL_PREFIX` to the path the example should be installed in
6. Click *Generate*
7. Click *Open Project*
8. Build *ALL_BUILD* and *INSTALL*

## Run

Running above build commands will build a library `rc_visard_opencv_example`
and an executable `rc_visard_show_streams`.

`rc_visard_show_streams` will show the received images in an OpenCV window.
One can cycle through the streams using by pressing `n` (next) or 
`p` (previous).
Press `q` to quit.

`rc_visard_show_streams` takes the optional flags 
`--left`, `--right`, `--disparity`, `--confidence`, `--error`
to select which image streams to show.
Using the flag `--synchronize` one can specify to receive only synchronized 
data sets. 

One also needs to specify the `device-id`, which is either the serial number,
the user defined ID or GenTL ID of the rc_visard.

### Linux

~~~bash
./rc_visard_show_streams [options] <device-id>
~~~

### Windows

Note: The PATH environment variable should point to the folders containing 
the DLLs of OpenCV and `rc_genicam_api`.

~~~
rc_visard_show_streams.exe [options] <device-id>
~~~
