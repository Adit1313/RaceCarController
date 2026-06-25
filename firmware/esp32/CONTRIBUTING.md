# Contributing

Welcome to the CRS Car Firmware Project! If you'd like to contribute to the development, here are some instructions on how to set up your development environment. In addition, please also mind the guidelines on how to format and document your code!

## Table of Contents

1. [Get the Code](#get-the-code)
2. [Setting up the development environment](#setting-up-the-development-environment)
   1. [Docker environment](#docker-environment)
   2. [Native environment](#native-environment)
      1. [Basics](#basics)
      2. [Pre-Commit](#pre-commit)
      3. [Doxygen](#doxygen)

## Get the Code

To get the code from this repository, clone it to a location of your choice:

```bash
cd ~/git						# example location
git clone git@gitlab.ethz.ch:ics-group/students/lukas-vogel/crs-wifi-car-firmware.git
```

The repository also contains the correct version of the ESP-IDF toolchain that you need to build against. It is contained in a submodule, so change to your project folder and clone the submodules as well:

```bash
cd crs-wifi-car-firmware
git submodule update --init --recursive
```



## Setting up the development environment

To do development on this project, you have two options:

1. **Docker** environment: Work within a containerized environment, not one worry about dependencies. Note: USB Flashing is **not** possible inside Docker on macOS or Windows!
2. **Native** environment: Get the best performance, and some additional features (USB flashing!)



### Docker environment

This repository already contains a Dockerfile, which can be used to set up a development environment that contains all the dependencies. You will not have to install anything (besides Docker and an editor), and building the project should work out-of-the-box. However, the performance will be lower if the environment is emulated (ARM-based Macs for example), and you can not flash the software through the serial ports if you're not on Linux.

#### VS Code Development Container

* Open the project folder in Visual Studio Code
* Visual Studio Code will automatically prompt you to "re-open the folder inside a container" if you've got Docker installed.
* The Docker container is built for you using `docker-compose` and the ESP-IDF toolchain is installed.

#### Standalone container

If you prefer working with a different editor, you can also use a Docker container to build the firmware.

* Navigate to the project folder, e.g.

  ```bash
  cd ~/git/crs-wifi-car-firmware
  ```

* Run `build.sh`:

  ```bash
  ./build.sh
  ```

This will build the toolchain and output the binaries in the `build/` folder.


### Native environment

Using this setup, you will have to install some additional dependencies, but you also get the best performance. The required components for developing are:

* [All the dependencies](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#step-1-install-prerequisites) of ESP-IDF, which include:
  * macOS: `cmake`, `ninja`, `dfu-util`, `python3`. Check out the [guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/macos-setup.html) from ESP-IDF.
  * Linux: depends on your distro. There's also a [guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-setup.html) for you.
  * Windows: Follow [the Windows guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html).
* `pre-commit`, if you want to commit code[^1]
* `doxygen`, if you want to generate documentation[^2]

#### Basics

**Important**: Most of the dependencies should be automatically installed with the following script. If it does not work, go through the above checklist and install the mentioned dependencies manually. Here's the guide:

If you followed the instructions in the ["Get the Code" section](#get-the-code), you already have a repository that includes the entire toolchain you need. In the project root folder, **source** the install script as follows:

```bash
source ./setup native
```

This installs the appropriate compilers for your platform. You need to run this *at least once*, so that the tools are installed for graphical editors (VS Code).

However, if you do development in the terminal, you need ot run this *each time* you open a new shell, or, optionally, add it to your `.bashrc` (or `.zshrc` etc.).

If you want to commit code to the repository, instead of just flashing the firmware to the cars, continue. Otherwise you're done.

#### Pre-Commit

Pre-commit is used to ensure that all code committed to the repository is automatically formatted according to some standards (clang-format, line endings, etc.). It runs when you commit, and it will format the files in the process. This however will abort the commit, if there were any changes. Just commit again, and the newly formatted files are committed just as usual.

##### Installation

*Pre-commit is already installed in the Docker environment! These instructions only apply for a native install.*

To install [pre-commit](https://pre-commit.com), you can use pip

```shell
pip install pre-commit
```

or homebrew:

```shell
brew install pre-commit
```

or something else, just read the [official instructions](https://pre-commit.com/#install). After the tool was installed, run

```shell
pre-commit install
```

in the *project root folder*. This will automatically install the git hooks for you. Now, each time you commit, the files will be automatically formatted.

#### Doxygen

Follow the [installation guide](https://www.doxygen.nl/download.html) on the Doxygen website.



## Debugging

To debug the code interactively, the [native ESP-IDF installation](#native-environment) is required. After installing a local copy of ESP-IDF, perform the following steps:

1. Install [OpenOCD](https://openocd.org/pages/getting-openocd.html). For Ubuntu and macOS, this can be done with the following commands:
```shell
  # Ubuntu
  sudo apt-get install -y openocd
  # macOS
  brew install openocd
```
2. Verify the installation by running `openocd --version`.
3. Start the OpenOCD Server through the VS Code command palette (Ctrl+Shift+P) by typing "ESP-IDF: OpenOCD Manager".
4. Launch a debug session from the "Launch & Debug" tab in VS Code.

Debugging currently only works in the native environment and in VS Code. Connect the ESP-PROG device to the JTAG port of the PCB, then choose "ESP-IDF: Flash (with JTAG)". If flashing succeeds, you can get to debugging!

## Footnotes

[^1]: You absolutely *can* commit code even without `pre-commit`, but @vogell won't be happy about your not-nicely-formatted files.
[^2]: Again: you don't *have* to document your code for the project, it would be nice though.
