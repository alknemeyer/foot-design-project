#! /usr/bin/fish
# chmod +x start.fish

# fix permission errors from serial <-> OptoForce
sudo chmod 666 /dev/ttyACM0

# script on my computer that activates conda. I do this
# manually to stop the constant (base) thing
source-conda;

and conda activate foot-design;

# if the script is run as `./startfish i`,
# launch ipython
and if test "$argv[1]" = "i"
    ipython
end