# MK III Code
The official repository for all code on the MK III Car for Olin Electric Motorsports

## Setting Up
To install all the necessary dependencies and liraries, run the `setup.sh` script below.
This already knows what you need to download, so it does all the work for you. All you need to do is type `y` when prompted to allow it to download.
```
$ bash setup.sh
```

Once you've run the setup script, make a folder in the 'boards' folder with the name of your board. Once you're in your folder, make a C document named something like `boardname.c` or `main.c`. 

## How to Flash a Board
Once you've designed your fancy PCB, you probably want to write some code that goes on it. Here's how you do it:
-Navigate to the head of the MK_III-Code repo.
```
$ python3 make.py
```
-Type in the name of the board you want to flash. The name must match exactly to the name of the folder containing the board.
```
$ Board (i.e. Dashboard): Blinky
```
-Select if you want to flash it or if you want to set the fuses.
```
$ Flash (y/n) or Set Fuses(fuses): y
```
You should see a whole bunch of information fly across your screen. This is the AVRDude reading back to you what is going on. You can ignore it, unless stuff breaks, then it's nice to look back and see what broke and when.

If you're curious as to what is actually going on 'behind the scenes', I've written a short articles explaining how it.

## Tutorials
New to Formula or moved to a different subteam? Read and follow along with these tutorials and you can learn a whole bunch about electrical systems and embedded hardware. 

#### Learn C
Learn C [here...](https://github.com/olin-electric-motorsports/C_Tutorials)

#### Programing Embedded Hardware
Learn that [here...](https://github.com/olin-electric-motorsports/Programming_Tutorials)

#### Github
Learn how to git gud [here...](https://github.com/olin-electric-motorsports/Programming_Tutorials/tree/master/GitTutorials)


## Useful Links
-[How does Git/Github Work?](http://product.hubspot.com/blog/git-and-github-tutorial-for-beginners)
- []

-[Getting started in C](https://www.programiz.com/c-programming)

-[Markdown Tutorial and Tips](https://guides.github.com/features/mastering-markdown/)
