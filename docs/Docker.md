# Docker Usage
This Dockerfile assumes that your Rust code is located in the current directory and that the binary is named `my_binary`. You can customize the `CMD` instruction to run your specific binary.

To build the Docker image, navigate to the directory containing the `Dockerfile` and run the following command:

```bash
docker build -t my_image .
```


This will build the Docker image and tag it with the name `my_image`. You can then run the Docker container using the following command:

```bash
docker run my_image
```
This will start the container and run the Rust binary inside it.

Or you can run it while open a terminal connection into the container. 

```bash
docker run -it my_image
```

# Installation
1. Follow official instruction from [docker](https://docs.docker.com/desktop/install/ubuntu/)
2. `sudo groupadd -f docker`
3. `sudo usermod -aG docker $USER`
4. `newgrp docker`
5. `groups`