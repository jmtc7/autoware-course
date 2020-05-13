# Setup Guide
Once having a working installation of Agile Development Environment (ADE) (and, therefore, of Docker Engine), ADE will need a ***home directory*** in the host machine. This directory will contain dotfiles and bust be different than the user's home directory **outside** of the container. e.g. the *home directory* that needs to be created could be `~/adehome`. In order to initialize it, it is necessary to create it and to generate an empty `.adehome` file inside of it.

```bash
$ mkdir adehome
$ cd adehome
$ touch .adehome
```

Autoware.Auto provides an [`.aderc` file](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/blob/master/.aderc) (use the hyperlink to download it), whose existance is essential for ADE to function. The default values can be overridden with environment variables. `ade --help` will provide further details. A default environment can be setted up by clonning the Autoware.Auto repository as follows:

```bash
$ cd adehome
$ git clone --recurse-submodules https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto
```

Next, to build Autoware.Auto, the following steps have to be followed:

```bash
$ ade start --update --enter 
ade$ cd AutowareAuto
ade$ colcon build
ade$ colcon test
ade$ colcon test-result
```

