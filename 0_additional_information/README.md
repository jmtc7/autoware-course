# Additional Information
In this folder, additional relevant information related to the course, ROS 2 and Autoware is contained.


## Introduction to Autoware. Why Autoware.Auto?
Autoware is a software stack maintained by [The Autoware Foundation](https://www.autoware.org/), which goal is to support open-source projects that will enable self-driving mobility. It has many members from both industrial and academical environments. In 2020, there are thee Autoware projects, which are:

- [**Autoware.AI**](http://www.autoware.ai/): It is based in ROS 1. It was the first Autoware project and its goal is to be a R&D platform for autonomous driving. Its main use is in academia.
- [**Autoware.IO**](http://www.autoware.io/):: An interface for Autoware to be extended with proprietary software and third-party libraries. It contains sensor drivers, by-wire controllers, hardware-dependent programs for System on Chip (SoC) boards, etc.
- [**Autoware.Auto**](http://www.autoware.auto/):: Somehow an evolution of Autoware.ai. It is based in ROS 2, which gives it Real-Time (RT) capabilities and more security measures that makes it more trustable for real world critical applications. It is being freshly developed to make sure that the best development practices are used, so it will not only be a quick adaptation of Autoware.ai. Further details on how this migration from ROS 1 to ROS 2 is done can be found [here](https://www.apex.ai/post/porting-algorithms-from-ros-1-to-ros-2). The following are some relevant concepts kept in mind during the migration:
  - Usage of the **mulithreading** only when it is needed, but in every case it will be useful.
  - Interruption-based **polling** instead of infinite polling to avoid jitter.
  - Keep the **concers separated**. In particular, they will be (as suggested [here](https://limo.libis.be/primo-explore/fulldisplay?docid=LIRIAS1748457&context=L&vid=Lirias&search_scope=Lirias&tab=default_tab&lang=en_US&fromSitemap=1): computation, communication, coordination, composition and configuration. Even some of those are a ROS task, the developers have kept it in mind to separate the concers in their power.
  - **Safety-critical coding** standards linked with data and time deterministic systems. Some of the existing standards are: [Autosar Standard](https://www.autosar.org/fileadmin/user_upload/standards/adaptive/17-10/AUTOSAR_RS_CPP14Guidelines.pdf), [MISRA C/C++ Standard](https://www.misra.org.uk/Activities/MISRAC/tabid/171/Default.aspx) and [JSF-AV](http://www.stroustrup.com/JSF-AV-rules.pdf).
  - Avoiding **dynamic memory allocation** to avoid potential runtime problems.
  - Avoiding **blocking calls** so that they will not make the application undeterministic.
  - **Error handling and reporting**.
  - **Unit testing** ov every function, line, branch, and condition of the code to ensure its correctness.


## Autoware.Auto Installation and Setup
This section is a self-contained guide on the [general tutorial](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-and-development.html) provided in the Autoware.Auto documentation. This will assume a *x86_64 Ubuntu* system with an NVidia GPU.

### ADE Installation
[ADE](https://ade-cli.readthedocs.io/en/latest/) is a modular Docker-based tool to ensure the same environment among different systems. The official Docker installation process for Ubuntu systems can be found [here](https://docs.docker.com/engine/install/ubuntu/#installation-methods). The first step will be to **uninstall old versions** if any one is installed, they can be uninstalled by using the following command (since they were called `docker`, `docker.io`, or `docker-engine`):

```bash
$ sudo apt-get remove docker docker-engine docker.io containerd runc
```


The next thing to do is to **set up the Docker's official repository**. This will consist in (1) install tools to allow `apt` to use a repository over HTTPS, (2) add Docker's official GPG key (and verify it), and (3) set up the actual repository.

The installation of the tools to allow `apt` to use HTTPS is done by running:

```bash
$ sudo apt-get update

$ sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
```


Next, to add the Docker's GPG key:

```bash
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

It should be compared to the fingerprint `9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88`. This is done by using the `apt-key fingerprint` command to search for the last 8 characters of the fingerprint as follows:

```bash
$ sudo apt-key fingerprint 0EBFCD88                                                                                                                                                                                              
pub   rsa4096 2017-02-22 [SCEA]
      9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88
uid           [ unknown] Docker Release (CE deb) <docker@docker.com>
sub   rsa4096 2017-02-22 [S]
```


The final part of this step is to actuall add the repository, which (for x86 Ubuntu systems) is done as follows:

```bash
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```

Note that the last line installs the *stable* repository, the *nightly* and *test* ones can be also added by adding these words after the `stable` one in the previous command. More information about them can be found [here](https://docs.docker.com/engine/install/). It is also relevant to mention that the `$(lsb_release -cs)` part will return the name of the Ubuntu distribution (such as `xenial`. Some Ubuntu flavours may have some problems with this, so it may be necessary to directly write the used parent Ubuntu distribution (e.g. for `Linux Mint Tessa`, it should be substituted by `bionic`).


The last step is to actually **install the Docker Engine** using the added repository. This is done as follows:

```bash
$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Finally, in order to **check the installation**, the `hello-world` image can be used, which should output something similar to which is shown below.

```bash
$ sudo docker run hello-world

Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

In order to uninstall Docker Engine, the Docker Engine, CLI and Contained packages would have to be uninstalled as follows:

```bash
$ sudo apt-get purge docker-ce docker-ce-cli containerd.io
```

The images, containers, volumes or customized configuration files are not automatically removed. Even the customized configurations files will have to be manually removed, the images, containes, and volumes can be removed from the system using:

```bash
$ sudo rm -rf /var/lib/docker
```


### ADE Installation
After having an operative installation of Docker, it is possible to continue with the Agile Development Environment (ADE) installation, as explained in this [guide](https://ade-cli.readthedocs.io/en/latest/install.html). The first step is to **download the statically-linked binary** from the [releases page](https://gitlab.com/ApexAI/ade-cli/-/releases) of the `ade-cli` project. The [`ade+x86_64`](https://gitlab.com/ApexAI/ade-cli/uploads/85a5af81339fe55555ee412f9a3a734b/ade+x86_64) is the appropiate binary for the assumed system.

The next step is to **install it in the path** of the machine where it is being installed. To do so, it is convenient to rename it to `ade` and install somewhere in the system's `$PATH` variable. In Ubuntu, it is recommended to use `/usr/local/bin`. The `mv` command can be used for this task by using:

```bash
$ sudo mv ade /usr/local/bin/
```

Next, the binary should be given **execution permissons**, which is done with `chmod` as follows:

```bash
$ cd /usr/local/bin/
$ sudo chmod +x ade
```

Finally, **the installation can be checked** by executing from wherever in the system the following (which should provide similar outputs to the ones shown:

```bash
$ which ade
/usr/local/bin/ade

$ ade --version
4.1.0
```


In order to update `ade-cli`, `ade update-cli` should be ran. It will ask for confirmation if a new version is available before downloading and replacing it. 

To enable autocompletion, the following should be added to the `.zshrc` or `.bashrc` file of the system:

```bash
if [ -n "$ZSH_VERSION" ]; then
    eval "$(_ADE_COMPLETE=source_zsh ade)"
else
    eval "$(_ADE_COMPLETE=source ade)"
fi
```
