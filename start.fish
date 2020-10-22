#! /usr/bin/fish
# chmod +x start.fish

sudo chmod 666 /dev/ttyACM0

source-conda;

and conda activate foot-design;

and if test "$argv[1]" = "i"
    ipython
end