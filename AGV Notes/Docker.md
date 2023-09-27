# Docker 
1. Resources
	- [Docker Intro + Tutorial](https://www.youtube.com/watch?v=pTFZFxd4hOI&t=1330s)
	- [Docker in 100s](https://www.youtube.com/watch?v=Gjnup-PuquQ)
	- [Docker installation](https://www.youtube.com/watch?v=K03beiGemKQ) 
	- [Docker Docs](https://docs.docker.com/)
	- [Slides](https://docs.google.com/presentation/d/1nzzvHQuBJAqdjlPDUl0l2_rU-MAu8jctu1Gt66vlHHM/edit#slide=id.g10838dd9226_0_15)
	- [FreeCodeCamp](https://www.freecodecamp.org/news/the-docker-handbook/)
 
2. Steps to follow to create an image of a folder
	1. Create a `Dockerfile` without an extension inside the directory we want an image of.
	2. Run the commnad `docker build -t getting-started .` 
		- Builds the image
		- `-t` is the tag argument which tags the image into a human readable form of tag name `getting-started` 
		- `.` specifes that the Dockerfile is to be looked for in the current directory
	3.  The content of Dockerfile is :
 
		```
		# syntax=docker/dockerfile:1
		
		FROM node:18-alpine
		WORKDIR /app
		COPY . .
		RUN yarn install --production
		CMD ["node", "src/index.js"]
		EXPOSE 3000
		```
  
3. Running the Dockerfile
	1. `docker run -dp 127.0.0.1:3000:3000 getting-started` 
		-  The `-d` flag (short for `--detach`) runs the container in the background. The `-p` flag (short for `--publish`) creates a port mapping between the host and the container. The `-p` flag takes a string value in the format of `HOST:CONTAINER`, where `HOST` is the address on the host, and `CONTAINER` is the port on the container. The command publishes the container's port 3000 to `127.0.0.1:3000` (`localhost:3000`) on the host. Without the port mapping, you wouldn't be able to access the application from the host.
4. `docker ps` lists all containers currently present
5.  `docker build -t getting-started` can be used again to build an updated image (when we update the code of the application)
6. Removing a container:
	- Get the ID of the container using `docker ps`
	- `docker stop <the-container-id>` stops the container
	- `docker rm <the-container-id>` removes the container
 7. Hello
	-Hi 



## Docker freeCodeCamp
### Advantages of Docker
- Develop and run the application inside an isolated environment (known as a container) that matches your final deployment environment.
- Put your application inside a single file (known as an image) along with all its dependencies and necessary deployment configurations.
- And share that image through a central server (known as a registry) that is accessible by anyone with proper authorization.
## Installation
- [Link](https://docs.docker.com/engine/install/ubuntu/)
## Commands
1. `docker ps -a` to have a look at all the containers that are currently running or have run in the past
```
docker ps -a

# CONTAINER ID        IMAGE               COMMAND             CREATED             STATUS                     PORTS               NAMES
# 128ec8ceab71        hello-world         "/hello"            14 seconds ago      Exited (0) 13 seconds ago                      exciting_chebyshev
```
2. `container ls ` lists out *currently* running containers 
```
docker container ls

# CONTAINER ID        IMAGE                 COMMAND                  CREATED             STATUS              PORTS                  NAMES
# 9f21cb777058        fhsinchy/hello-dock   "/docker-entrypoint.…"   5 seconds ago       Up 5 seconds        0.0.0.0:8080->80/tcp   gifted_sammet
```
## Container
  
> A container is an abstraction at the application layer that packages code and dependencies together. Instead of virtualizing the entire physical machine, containers virtualize the host operating system only.

 ![virtual-machines](https://www.freecodecamp.org/news/content/images/2021/04/virtual-machines.svg)
 - Unlike a virtual machine, a container does the job of virtualization in a smarter way. Instead of having a complete guest operating system inside a container, it just utilizes the host operating system via the container runtime while maintaining isolation – just like a traditional virtual machine.

## Images
- Images are multi-layered self-contained files that act as the template for creating containers. They are like a frozen, read-only copy of a container. Images can be exchanged through registries.
> Containers are just images in running state.

## Registry
- An image registry is a centralized place where you can upload your images and can also download images created by others.

## Docker Architecture Overview
The engine consists of three major components:

1. **Docker Daemon:** The daemon (`dockerd`) is a process that keeps running in the background and waits for commands from the client. The daemon is capable of managing various Docker objects.
2. **Docker Client:** The client  (`docker`) is a command-line interface program mostly responsible for transporting commands issued by users.
3. **REST API:** The REST API acts as a bridge between the daemon and the client. Any command issued using the client passes through the API to finally reach the daemon.

![docker-run-hello-world](https://www.freecodecamp.org/news/content/images/2021/01/docker-run-hello-world.svg)

## Running a container 
```
docker <object> <command> <options>
```

In this syntax:

- `object` indicates the type of Docker object you'll be manipulating. This can be a `container`, `image`, `network` or `volume` object.
- `command` indicates the task to be carried out by the daemon, that is the `run` command.
- `options` can be any valid parameter that can override the default behavior of the command, like the `--publish` option for port mapping.

**Example**
```
docker container run --publish 8080:80 fhsinchy/hello-dock
```
1. Runs a container hello-dock built by fhsinchy such that request sent to port 8080 of thenhost system willl be forwarded to port 80 inside the container
## Renaming a container 
Every container has two identifiers.
- `CONTAINER ID` - a random 64 character-long string
- `NAME` - combination of two random words, joined with an underscore
1. Arguments
	- `--publish` - Specifies the port from which reuqest has to be forwarded from and to
	- `--name` Renames the container
## Running a container in interactive mode
- Some images do not just run some pre-configured program. These are instead configured to run a shell by default.
- In case of the operating system images it can be something like `sh` or `bash` and in case of the programming languages or run-times, it is usually their default language shell.
- These images require a special `-it` option to be passed in the `container run` command.
- The `-it` option sets the stage for you to interact with any interactive program inside a container. This option is actually two separate options mashed together.
	- The `-i` or `--interactive` option connects you to the input stream of the container, so that you can send inputs to bash.
	- The `-t` or `--tty` option makes sure that you get some good formatting and a native terminal-like experience by allocating a pseudo-tty.
 
## Starting a container again
- `docker container start <container identifier>` . Where container identifier is CONTAINER ID or CONTAINER NAMES
- `container ls --all` : Getting list of all containers. Then look for containers with `Exited` status
