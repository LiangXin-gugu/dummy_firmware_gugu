#ifndef __ASCII_PROTOCOL_H
#define __ASCII_PROTOCOL_H


/* Includes ------------------------------------------------------------------*/
#include <fibre/protocol.hpp>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink& response_channel);
void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);
void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);
void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);

// Function to send messages back through specific channel (UART or USB-VCP).
// Use this function instead of printf because printf will send messages over ALL CHANNEL.
template<typename ... TArgs>
void Respond(StreamSink &output , const char *fmt, TArgs &&... args)
{

    // Combine message and CRLF into a single buffer so that process_bytes
    // (and the underlying DMA transfer) sends them atomically. Previously the
    // two separate process_bytes calls could be interleaved by another thread's
    // Respond on the same channel, producing concatenated output like
    // "ok queued free=15ok\r\n\r\n".
    char response[66]; // 64 for message + 2 for CRLF
    size_t len = snprintf(response, sizeof(response) - 2, fmt, std::forward<TArgs>(args)...);
    response[len] = '\r';
    response[len + 1] = '\n';
    output.process_bytes((uint8_t *) response, len + 2, nullptr);
}


#endif /* __ASCII_PROTOCOL_H */
