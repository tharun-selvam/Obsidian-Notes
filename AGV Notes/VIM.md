## Resource
1. [Link](https://linuxconfig.org/vim-tutorial)

## Basics
1. Moving around:
	- `h` or `left arrow key` = move cursor left
	- `l` or `right arrow key` = move cursor right
	- `k` or `up arrow key` = move cursor up
	- `j` or `down arrow key` = move cursor down
 2. Command Mode: 
	- Command mode is the one where we can instruct Vim editor to exit to the command line ( shell ).
	- Press `ESC` to enter command mode 
 3. Editing mode:
	- In order to insert some text, you need to switch Vim into the editing mode. 
	- To get into editing mode press `i` while currently in the command mode.
	- `— INSERT —` keyword will appear at the bottom left of the screen and indicates that you are in insert mode.
 
 3. Exiting from Vim: To do that we need press `ESC` and type `:q!`.
	- The `:` character prefaces our command
	- The `q` will quit out of Vim
	- The `!` character instructs Vim to not save changes
 
 4. Deleting:
	 - Position on the character and press `x`
  
 5. Inserting:
	 - Enter into edit mode

  6. Saving edited file:
	- Use `:wq` command in Vim command mode.
		- The `w` command will write (save) changes
		- The `q` will quit out of Vim

## Summary
- **Moving cursor around**: `h` = left, `l` = right, `k` = up, `j` = down
- **Exit without saving**: Press `ESC` for command mode, then enter `:q!` to exit
- **Deleting characters**: Highlight a character and press `x` in command mode
- **Insert or append text**: Press `i` or `a` in command mode and start typing
- **Exit and save changes**: `:wq` or `SHIFT+zz` in command mode
		  