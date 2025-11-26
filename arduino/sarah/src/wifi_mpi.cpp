#include "..\headers\wifi_mpi.h"
#include "..\headers\wifi_socket.h"
#include "nsapi_types.h"
#include "string.h"
#include <stdarg.h>
/// @brief facilitates communication between M4 and M7 cores. Should call RPC.begin on both
///cores before using this api
namespace WIFI_MPI
{
    void Setup()
    {
        setup_lucas_client();
        //setup wifi
    }
    /// @brief Sends binary data through RPC serial M4/7
    /// @param code the code so the receiving side knows what to do. If this is 's' it will be
    /// treated as a string write and pass as plain text
    /// @param data pointer to the data
    /// @param length length of the data
    bool Send(const unsigned char code, const void* data,const unsigned int length)
    {
        while(true)
        {            
            //send over wifi
            if(!send_bytes(&code,1))
            {
                Serial.println("1");
                tryConnectToServer();
                continue;
            }
            
            if(!send_bytes((unsigned char*)&length,4))
            {
                Serial.println("2");
                return false;
            }
            if(length>0 && data!=NULL)
            {
                if(!send_bytes((unsigned char*)data,length))
                {
                    Serial.println("3");
                    return false;
                }
            }
            delay(5);
            return true;
        }
    }
   bool Send(const unsigned char code)
   {
        return Send(code,NULL,0);
   }

   void Printf(const char *format, ...)
{
    char buf[256];             // adjust size if needed
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);  // format string with params
    va_end(args);

    Send(TEXT_STREAM_OUT,buf, std::strlen(buf));       // send to Serial
}

   bool Print(std::string s)
   {
        return Send(TEXT_STREAM_OUT,s.c_str(),s.length());
   }

    /// @brief sends a message through the RPC serial stream
    /// @param m the message to send
    bool SendMessage(Message m)
    {
        return Send(m.code,m.data,m.length); 
    }

    void(*functionPtrs[256])(Message&);

    /// @brief registers a message handler. Note; 's' is reserved for text based streams and
    /// should generally be registered to forward to Serial.print*
    /// @param code the code to register the message handler against
    /// @param functionPtr pointer to the handler function
    void RegisterMessageHandler(unsigned char code, void(*functionPtr)(Message&))
    {
        functionPtrs[code] = functionPtr;
    }

    bool HandleMessage(Message& m)
    {
        if(m.code>0)
        {
            void(*functionPtr)(Message&) = functionPtrs[m.code];
            if(functionPtr)
            {
                functionPtr(m);
                return true;
            }
        }
        return false;
    }

    void ProcessMessage(unsigned char code)
    {
        Message m;
        unsigned char buffer[128];
        //read over wifi
        m.code =code;
        
        if (m.code >= 0)
        {           
            //read over wifi
            get_bytes((unsigned char*)&m.length,4);
            if(m.length > 0)
            {
                Serial.println(m.length);
                get_bytes(buffer,m.length);
                m.data = buffer;
            }
            HandleMessage(m);
        }
    }

    /// @brief reads and processes by handlers all of the messages in the RPC stream
    void ProcessMessages()
    {     
        while (true) 
        {
            uint8_t buf[1];
            nsapi_size_or_error_t n =get_bytes(buf, sizeof(buf));

            if (n == NSAPI_ERROR_WOULD_BLOCK) {
                break;            
            } else if (n < 0) {
                Serial.println("closing");
                close();
                tryConnectToServer();
                return;
            } else {
                ProcessMessage(buf[0]);
                break;    
            }

        }
    }
}
