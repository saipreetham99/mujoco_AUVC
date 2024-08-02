#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 12345
#define IP "127.0.0.1"  // Localhost IP address

int main() {
    int sockfd;
    struct sockaddr_in servaddr;
    const int arraySize = 6;
    float data[arraySize] = {1.1, 2.2, 3.1, 4.2, 5.3, 6.4};  // Example array to send

    // Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed." << std::endl;
        return -1;
    }

    // Prepare server address
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr(IP);

    // Send array data
    if (sendto(sockfd, data, sizeof(data), 0, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Send failed." << std::endl;
        close(sockfd);
        return -1;
    }

    std::cout << "Array sent successfully." << std::endl;

    // Close socket
    close(sockfd);
    return 0;
}

