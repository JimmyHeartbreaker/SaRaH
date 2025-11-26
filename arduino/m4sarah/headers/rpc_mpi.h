#ifndef RPC_MPI_H
#define RPC_MPI_H
#include "RPC.h"
#include <string>

/// @brief facilitates communication between M4 and M7 cores. Should call RPC.begin on both
///cores before using this api
namespace RPC_MPI
{
    struct Message
    {
        char code;
        char* data;
        short length;
    };
    const char SERIAL_OUT = 0x01;

    
    void Setup();
    
    /// @brief Sends binary data through RPC serial M4/7
    /// @param code the code so the receiving side knows what to do. If this is 's' it will be
    /// treated as a string write and pass as plain text
    /// @param data pointer to the data
    /// @param length length of the data
    void Send(const char code, const char* data,const unsigned int length);

    /// @brief prints the data to the RPC serial stream. 's' code is prepended to the stream.
    /// Appends '\0' to the end if it is not present
    /// @param data pointer to the data
    /// @param length length of the data
    void Print(const char* data,const unsigned int length);

    /// @brief prints a string to the RPC serial stream. 's' code is prepended to the stream
    /// @param s string reference
    void Print(const std::string &s);

    /// @brief prints an integer to the RPC serial stream. 's' code is prepended to the stream
    /// @param i the value to print
    void Print(const int i);

    /// @brief sends a message through the RPC serial stream
    /// @param m the message to send
    void SendMessage(Message m);


    /// @brief registers a message handler. Note; 's' is reserved for text based streams and
    /// should generally be registered to forward to Serial.print*
    /// @param code the code to register the message handler against
    /// @param functionPtr pointer to the handler function
    void RegisterMessageHandler(char code, void(*functionPtr)(Message&));

    bool HandleMessage(Message& m);

    void ProcessMessage();

    /// @brief reads and processes by handlers all of the messages in the RPC stream
    void ProcessMessages();
}

#endif