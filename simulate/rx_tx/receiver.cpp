#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 12345

int main() {
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    socklen_t len;
    const int arraySize = 6;
    float buffer[arraySize];

    // Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed." << std::endl;
        return -1;
    }

    // Prepare server address
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket to the address
    if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Bind failed." << std::endl;
        close(sockfd);
        return -1;
    }

    std::cout << "Waiting for data..." << std::endl;

    // Receive data
    len = sizeof(cliaddr);
    int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);
    if (n < 0) {
        std::cerr << "Receive failed." << std::endl;
        close(sockfd);
        return -1;
    }

    // Print received array
    std::cout << "Received array: ";
    for (int i = 0; i < arraySize; ++i) {
        std::cout << buffer[i] << " ";
    }
    std::cout << std::endl;

    // Close socket
    close(sockfd);
    return 0;
}
