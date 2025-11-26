#ifndef WIFI_MPI_H
#define WIFI_MPI_H
#include <string>
#include "string.h"

/// @brief facilitates communication between M4 and M7 cores. Should call RPC.begin on both
///cores before using this api
namespace WIFI_MPI
{
    struct Message
    {
        unsigned char code;
        int length;
        void* data;
    };
    const char TEXT_STREAM_OUT = 0xFF;
    

    
    void Setup();
    
    /// @brief Sends binary data through RPC serial M4/7
    /// @param code the code so the receiving side knows what to do. If this is 's' it will be
    /// treated as a string write and pass as plain text
    /// @param data pointer to the data
    /// @param length length of the data
    bool Send(const unsigned char code, const void* data,const unsigned int length);
    bool Send(const unsigned char code);
    void Printf(const char *format, ...);
    bool Print(std::string s);

    /// @brief sends a message through the RPC serial stream
    /// @param m the message to send
    bool SendMessage(Message m);


    /// @brief registers a message handler. Note; 's' is reserved for text based streams and
    /// should generally be registered to forward to Serial.print*
    /// @param code the code to register the message handler against
    /// @param functionPtr pointer to the handler function
    void RegisterMessageHandler(unsigned char code, void(*functionPtr)(Message&));

    bool HandleMessage(Message& m);

    void ProcessMessage(unsigned char code);

    /// @brief reads and processes by handlers all of the messages in the RPC stream
    void ProcessMessages();
}

#endif