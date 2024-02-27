# Scotland Yard

This is an auxillary program for my masters thesis. It aims to visualize information about a game state of a (lazy) cops vs. robber game. Cops vs. robber is a very cut down version of the board game scotland yard:
* there are two players. One controls the robber character, the other a fixed number of cop characters.
* the game starts with the police player placing his characters on the map, followed by the robber player placing his character on the map
* each following turn the cop player can move one of his cops to a vertex neighboring the cop's curent position or yield without move. then the robber player can move his robber character with the same rules.
* the police win, when they manage to move a cop character to the same spot as the robber.
* the robber wins, if he can evade indefinitely.

This variant of cops vs robbers is called lazy cops, because the police may only move a single cop in a turn.
It is unknown wether there exist planar graphs which when used as game map allow the robber to win against four or more lazy cops. This program tries to aid in the search for these maps.

### Testing locally

Make sure you are using the latest version of stable rust by running `rustup update`.

`cargo run --release`

On Linux you need to first run:

`sudo apt-get install libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libxkbcommon-dev libssl-dev`

On Fedora Rawhide you need to run:

`dnf install clang clang-devel clang-tools-extra libxkbcommon-devel pkg-config openssl-devel libxcb-devel gtk3-devel atk fontconfig-devel`

### Website Version

 Try it out at <https://brunizzl.github.io/scotland-yard-egui/>.



### TODO

#### Theory
* extend convex hull to prevent jumps
* think about how to extend the escapable regions concept to identify more vertices
* formulate robber strategy when outside hull

#### Functionality
* adapt hull boundary to respect torus (allow multiple disjoined boundary segements)
* fix bug where sometimes a cop in lazy-safe region is not regarded as such
* choose for which cops convex hull is computed + option to define the outside region yourself

#### Nice To Have
* pick colors for escapable regions depending on boundary cops (fewer color swaps) disadvantage: we only have 32 and want to avoid collisions at all cost (look up perfect hashing maybe?). 
* ability to delete any cop (not just the one added last)
* extract base functionality of EdgeList to own thing

#### Lowest priority
* platonic symmetry as own type (way faster to compute than explicit, only switch to explicit in bruteforce)

