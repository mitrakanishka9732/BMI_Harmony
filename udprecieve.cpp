#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#define PORT 8080
#define MAX_BUFFER_SIZE 1024

int main() {
    int sockfd, buffer_len;
    char buffer[MAX_BUFFER_SIZE];
    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "socket creation failed" << std::endl;
        return 1;
    }
    else{
      std::cout << "Socket succesflly created!" << std::endl;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "bind failed" << std::endl;
        return 1;
    }
    else{
      std::cout << "Binded succesflly!" << std::endl;
    }
    //allways be recieveing 
    while(true){
      // Receiving data from the client
      buffer_len = sizeof(cliaddr);
      int n = recvfrom(sockfd, (char *)buffer, MAX_BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&cliaddr, (socklen_t*)&buffer_len);
      buffer[n] = '\0';
      std::cout << "Client: " << buffer << std::endl;
    }

    close(sockfd);


    return 0;
}