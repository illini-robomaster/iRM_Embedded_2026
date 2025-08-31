# iRM Embedded 2023

![arm](https://github.com/illini-robomaster/iRM_Embedded_2022/workflows/arm%20build/badge.svg)

Embedded system development @ Illini RoboMaster

## User Guide (Linux / Mac)

You can follow the instructions below to setup the necessary environments for
building the source code and flashing the embedded chips.

### Install ARM Toolchain

1. Go to the [official download page](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
   for ARM Toolchain.
2. Download the pre-built toolchain according to your operating system.
3. Decompress it to some directory and find an absolute path to the `bin` directory.

   In my case: `/Users/alvin/gcc-arm-none-eabi-10.3-2021.10/bin`.
4. For Linux / Mac users, add the toolchain to your path:
   ```sh
   chmod -x /Users/alvin/gcc-arm-none-eabi-10.3-2021.10/bin/*
   sudo cp /Users/alvin/gcc-arm-none-eabi-10.3-2021.10/bin/* /usr/local/bin
   rm -rf /Users/alvin/gcc-arm-none-eabi-10.3-2021.10  # Optional
   ```
5. If you do not have root access: For Linux / Mac users, add the following line
   (replace `<path>` with the actual binary path found in step 3) to `~/.bashrc` for bash users
   or `~/.zshrc` for zsh users.
   ```sh
   echo 'export PATH=<path>:$PATH' > .bashrc  # or .zshrc etc.
   ```

### Compile Project

1. Go to your project root directory in a terminal.
2. Run the following command to build the entire project.

   ```sh
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make -j
   ```

   Change build type to `Debug` or `RelWithDebInfo` in order to debug with `gdb`.
   Note that `Debug` build could be much slower than the other two due to lack
   of compiler optimizations.

### Flash Binary to Chip

1. Install [stlink](https://github.com/stlink-org/stlink).

   1. For Mac users, `brew install stlink` could be a shortcut.
   2. For Ubuntu users, `sudo apt install stlink-tools` could be a shortcut.
   3. For Arch users, `sudo pacman -S stlink` could be a shortcut.
   4. For Linux users, either use prebuilt binaries, or build from source
      following their compile manual.

2. Flash one of the example programs by running `make flash-<xxx>` in the
   `build/` directory created at compilation.

   e.g. `make flash-example_buzzer` -> and you shall hear some music (or noise)

### Document Usage

You will need [Doxygen](https://www.doxygen.nl/index.html).

   1. For Mac users, `brew install doxygen` could be a shortcut.
   2. For Ubuntu users, `sudo apt install doxygen` could be a shortcut.
   3. For Arch users, `sudo pacman -S doxygen` could be a shortcut.
   4. For Linux users, either use prebuilt binaries, or build from source
      following their compile manual.

To generate documentations after compiling the project.

   - Run `make doc` in the `build/` directory

To view the generated document:

   - Run `firefox docs/html/index.html`

## Developer Guide

Use the following guide when making contributions to this repo.

### Format Code

The continuous integration system will check the source code against
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
All codes are required to be formatted correctly before merging. There are several
integrated build commands that can help you automatically format your changes.

**Prerequisite**: install `clang-format` (version 10 recommended)

* Linux users can simply install it using `sudo apt install clang-format-10`.
* Mac users need to download prebuilt binaries from
  [here](https://releases.llvm.org/download.html). For now, we **CANNOT**
  use version 11 or above.

With `clang-format` installed, you can run the following commands inside `build/`
to automatically format your changes.

1. `make check-format`: Check `diff` between current source and
   formatted source (without modifying any source file)
2. `make format`: Format all source files (**Modifies** file in place)

### Debug with `gdb`

To debug embedded systems on a host machine, we would need a remote gdb server.
There are 2 choices for such server, with tradeoffs of their own.

* **`st-util`**

  This tool comes with [stlink](#flash-binary-to-chip), but be aware
  that this is a third-party implementation and is not stable. The most
  recent release tested to be working is `v1.5.1` (**Notice:** `st-util`
  have poor performance, it could malfunction most of the time, so
  it is not recommended).

* **`OpenOCD`**

  This tool is much more stable but is slightly less intelligent in detecting
  ST-LINK version and it has not been updated since 2017. To install it,
    * `brew install openocd` for Mac users
    * `sudo apt install openocd` for Ubuntu users
    * `sudo pacman -S openocd` for Arch users

Follow the steps below to debug an executable

1. Launch a `gdb` server by either choice
    * `st-util`
    * `openocd -f <project root>/debug/OpenOCD/st-link-v2-1.cfg`
2. In a separate terminal, `cd` into the `build` directory and run `make debug-xxx`
   (e.g. `make debug-example_buzzer`). This will open up a `gdb` session.
3. Run `target extended-remote :<port>` (or `tar ext :<port>` in short )
   to connect to the `gdb` server.
    * For `st-util`, replace `<port>` with `4242`.
    * For `openocd`, replace `<port>` with `3333`.
4. Run `load` to flash the executable (Note that you can also run `make` here
   without exiting `gdb` to re-build the executable if you modified some
   source code).
5. Debug just like any regular `gdb` debugger: use commands like `continue`,
   `run`, `break`, `watch`, `next`, `step` the same way you will expect.

## Prerequisites before merging new changes to main
1. Generate a new ssh key if you don't have any (Follow the instruction of "Generating a new SSH key" from [this link](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)).
   
2. [Add your new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) to your Github account.

3. (You only need to complete this step if you cloned our repository using https instead of ssh).
   Go to your local repository directory, open `.git/config` and change `url=https://github.com/illini-robomaster/iRM_Embedded_2023.git` to
   `url=git@github.com:illini-robomaster/iRM_Embedded_2023.git`. 

4. Make sure you have created a [personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token).
   
5. Create a new branch from main and make all of your changes in that branch. After pushing it to the remote repository, create a pull request and assign at least one reviewer. After one reviewer approves your changes, your code can be merged to main. Learn more about pull requests [here](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests).
