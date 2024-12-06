# Save online maps and load them offline

## 1.Building:

1.1 **DONOT** forget to download the Vocabulary form the [origin repo](https://github.com/raulmur/ORB_SLAM2) and place it into dir ./Vocabulary

1.2 Create a folder based on the contents of `CMakeLists.txt`

1.3 build

```bash
     chmod +x build.sh
     ./build.sh
```

### only build the ORB_SLAM2 mode with pcl

```bash
    mkdir build
    cd build
    cmake ..
    make -j
```

## 2.Run:
* You can refer to the commands in the command.txt file

# What are modified:

* adding a pointcloud viewer with loopclosing ( realized by adding a viewer thread )
* save map
* load map

