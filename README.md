# Scotland Yard

This is an auxillary program originally written for my masters thesis. It aims to visualize information about a game state of a (lazy) cops vs. robber game. Cops vs. robber is a very cut down version of the board game scotland yard:
* there are two players. One controls the robber character, the other a fixed number of cop characters.
* the game starts with the police player placing his characters on the map, followed by the robber player placing his character on the map
* each following turn the cop player can move one of his cops to a vertex neighboring the cop's curent position. then the robber player can move his robber character with the same rules or stay put.
* the police win, when they manage to move a cop character to the same spot as the robber.
* the robber wins, if he can evade indefinitely.

### Testing locally

Make sure you are using the latest version of stable rust by running `rustup update`.

`cargo run --release`

On Linux you need to first run:

`sudo apt-get install libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libxkbcommon-dev libssl-dev`

On Fedora Rawhide you need to run:

`dnf install clang clang-devel clang-tools-extra libxkbcommon-devel pkg-config openssl-devel libxcb-devel gtk3-devel atk fontconfig-devel`

*NOTE: TO SAVE SCREENSHOTS / BRUTEFORCE DATA, YOU NEED TO HAVE THE FOLLOWING FOLDERS IN THIS DIRECTORY:*
- bruteforce
- bruteforce-police
- screenshots

### Website Version

 Try it out at <https://brunizzl.github.io/scotland-yard-egui/>.



### Nice To Have TODOs

* extract base functionality of EdgeList to own thing
* step away from EdgeList and towards BoolCsr
* (maybe to facilitate the above) add "Edges" trait that has the immutable part of the behavior of EdgeList 
* add randomly (with seed 👀) triangulated torus as graph option
* add randomly (with seed 👀) triangulated closed surface as graph option
* platonic symmetry as own type (way faster to compute than explicit, only switch to explicit in bruteforce)

