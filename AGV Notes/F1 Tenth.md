# F1 Tenth

## Docker 
1. Resources
	- [Docker Intro + Tutorial](https://www.youtube.com/watch?v=pTFZFxd4hOI&t=1330s)
	- [Docker in 100s](https://www.youtube.com/watch?v=Gjnup-PuquQ)
	- [Docker installation](https://www.youtube.com/watch?v=K03beiGemKQ) 
	- [Docker Docs](https://docs.docker.com/)
 

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
	- 