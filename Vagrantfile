###
# See: https://code-chronicle.blogspot.com/2014/08/connect-usb-device-through-vagrant.html
# See: https://docs.oracle.com/cd/E97728_01/E97727/html/vboxmanage-usbfilter.html
# Assuming MacOS as the host OS, with Homebrew already installed:
# Install Vagrant and Virtualbox with Homebrew:
#   brew cask install vagrant virtualbox virtualbox-extension-pack
#
# After installing Vagrant and Virtualbox, on the host machine,
# do:
#   VBoxManage list usbhost
# And note the USB device that matches your Arduino device, for example:
#   UUID:               <SOME UUID>
#   VendorId:           0x239a (239A)
#   ProductId:          0x800c (800C)
#   Revision:           1.0 (0100)
#   Port:               2
#   USB version/speed:  0/Full
#   Manufacturer:       Adafruit
#   Product:            Feather 32u4
#   Address:            <SOME COLLECTION OF IDENTIFIER NUMBERS>
#   Current State:      Busy
#
# Modify the `usbfilter` virtualbox customization below to set the
# `vendorid` field to be the hex-value of the VendorId, and
# set the `productid` to be the hex-value of the ProductId.
# This will share matching USB devices from the host through to the
# virtual machine.
#
# NOTE: this got complicated - in my case the device in bootloader mode
# has a different ProductId than it does in regular operation.  Experiment
# with various filters to make sure the Virtualbox instance can capture
# control of the USB device.
#
# After that, start the vagrant box and ssh into the machine with:
#  vagrant up
#  vagrant ssh
#
# Once in the instance, the project directory that contains the Vagrantfile
# on the host is shared into the virtual machine, mounted at /vagrant.
#
# From here, continue on to use the arduino-cli command from their documentation:
# See: https://github.com/arduino/arduino-cli#getting-started
# It's likely you should just become root and stay there, instead of operating as
# the vagrant user:
#   sudo su
#
# You'll want to be able to:
#   - detect your board
#   - verify you can talk to the board over a serial connection (screen, cat, etc)
#       - `cat /dev/ttyACM0` is a decent place to start
#   - load the core modules for your board
#   - load any libraries you need for your code
#       - if you're root, libraries can be manually placed in `/root/Arduino/libraries/`
#
# When it comes time to compile a sketch and load it, you can do something like:
#   arduino-cli compile -u -p /dev/ttyACM0 --fqbn adafruit:avr:feather32u4 blink
#
# Read the arduino-cli documentation for all the other things you can do from the
# command line.
#
# Good luck.
###


Vagrant.configure("2") do |config|
    config.vm.box = "ubuntu/bionic64"
    config.vm.box_check_update = true
    config.vm.boot_timeout = 300

    config.vm.provider "virtualbox" do |v|
        v.memory = "1024" # in MB
        v.customize ["modifyvm", :id, "--usb", "on"]
        v.customize ["modifyvm", :id, "--usbehci", "on"]
        v.customize ["usbfilter", "add", "0",
                     "--target", :id,
                     "--name", "Adafruit Feather 32u4",
                     "--vendorid", "239a",
                     "--productid", "800c",
                     "--remote", "no"]
        v.customize ["usbfilter", "add", "0",
                     "--target", :id,
                     "--name", "Adafruit Feather 32u4 - Programmer",
                     "--vendorid", "239a",
                     "--productid", "000c",
                     "--remote", "no"]
    end

    # Run the post-create script, defined down below
    config.vm.provision "shell", inline: $script

    # Enable/Disable random plugins I have installed
    # if you get errors about these (because you don't have them installed),
    # feel free to remove these lines.
    config.hostmanager.enabled = false  # for `vagrant-hostmanager` plugin
    config.vbguest.auto_update = true  # for `ansible-vbguest` plugin
end

# The quick/dirty provisioning script to run when the instance is created
$script = <<-SCRIPT
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -yq linux-image-extra-virtual

# The showports branch let's you see attached boards, even when they're not
# recognized.  Hopefully this makes it into the mainline branch, and we can
# go back to just installing the official release.
wget https://github.com/arduino/arduino-cli/releases/download/0.5.0-showports/arduino-cli_0.5.0-showports_Linux_64bit.tar.gz
tar -xvf arduino-cli_0.5.0-showports_Linux_64bit.tar.gz
ln -s /home/vagrant/arduino-cli /usr/local/bin/arduino-cli

# The `dialout` group controls access to the serial devices
adduser vagrant dialout
adduser root dialout
SCRIPT
