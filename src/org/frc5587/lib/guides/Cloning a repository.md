# Cloning Repositories

Because we us git submodules to carry our library files over year-to-year, cloning a repository takes a little more work

## Cloning with GitHub Desktop

You're in luck, you can just clone the repository normally and everything will work!

## Cloning through command line

### If you haven't cloned it yet

In the folder you want to repository to be placed in, run

```bash
git clone --recurse-submodules <Repository URL>
```

If you're unlucky

### If you already cloned it normally

If you forgot to add `--recurse-submodules` when cloning the repository, go to the directory of the repository and run

```bash
git submodule update --init
```

### Notes for command line

Regardless of the way you clone the repository, every time you try to pull updated code you will have to run

```bash
git pull --recurse-submodules
```

Also, it's pretty likely that something weird will happen with submodules at some point with command line because they're honestly just janky. So when in doubt, refresh the submodule with

```bash
git submodule update
```
