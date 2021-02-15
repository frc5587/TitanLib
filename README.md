# Java-Libs

This is a repository for all of the Java files that carry over year-to-year for FRC Team 5587, Titan Robotics. All of these files are free for use for another team or personal projects so long as use complies with the license in this repository.

## Generating Documentation

To generate documentation, simply use the `javadoc` command on your desired file:
```bash
$ javadoc -d ./docs/ <path_to_file>
```

Afterwards, open the `index.html` file found in the `docs/` folder however you wish.


## Add Java-Libs to a Repository as a Submodule

In the main repository:

1. Run `git init` in the vscode terminal

2. Create the directory path where you want the submodule to be located. This can be done either manually or with 
```bash 
$ mkdir -p <directory_location> 
(eg: mkdir -p src/main/java/org/frc5587/)
```

3. Add the submodule to the repository in the desired location: 
```bash
$ git submodule add https://github.com/frc5587/Java-Libs.git <directory_location/new_submodule_folder> 
(eg: git submodule add https://github.com/frc5587/Java-Libs.git src/main/java/org/frc5587/lib)
```

Note: the branch of the submodule repo can be switched through the git tab on the vscode sidebar.
