# Use the official Rust image as the base image
FROM ubuntu:latest

# Set the working directory 
VOLUME /home/motion_planning
WORKDIR /home/motion_planning

# Install curl for rustup
RUN apt-get -y update
RUN apt-get -y install \
    curl \
    build-essential

# Install rust and cargo
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Build the Rust code
#RUN cargo build --release

# Set the command to run the binary when the container starts
#CMD ["./target/release/my_binary"]

EXPOSE 3000