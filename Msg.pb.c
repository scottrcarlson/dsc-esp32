/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9.3 at Fri Sep  6 11:13:49 2019. */

#include "Msg.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t Msg_fields[18] = {
    PB_FIELD(  1, FIXED_LENGTH_BYTES, OPTIONAL, STATIC  , FIRST, Msg, content, content, 0),
    PB_FIELD(  2, INT32   , OPTIONAL, STATIC  , OTHER, Msg, originatorNodeId, content, 0),
    PB_FIELD(  6, INT32   , OPTIONAL, STATIC  , OTHER, Msg, originatorBattLevel, originatorNodeId, 0),
    PB_FIELD(  7, INT32   , OPTIONAL, STATIC  , OTHER, Msg, receiverRssi, originatorBattLevel, 0),
    PB_FIELD(  8, FLOAT   , OPTIONAL, STATIC  , OTHER, Msg, receiverSnr, receiverRssi, 0),
    PB_FIELD(  9, INT32   , OPTIONAL, STATIC  , OTHER, Msg, receiverFreqErr, receiverSnr, 0),
    PB_FIELD( 10, FLOAT   , OPTIONAL, STATIC  , OTHER, Msg, receiverLat, receiverFreqErr, 0),
    PB_FIELD( 11, FLOAT   , OPTIONAL, STATIC  , OTHER, Msg, receiverLng, receiverLat, 0),
    PB_FIELD( 15, FLOAT   , OPTIONAL, STATIC  , OTHER, Msg, senderLat, receiverLng, 0),
    PB_FIELD( 16, FLOAT   , OPTIONAL, STATIC  , OTHER, Msg, senderLng, senderLat, 0),
    PB_FIELD( 17, FLOAT   , OPTIONAL, STATIC  , OTHER, Msg, originatorLat, senderLng, 0),
    PB_FIELD( 18, FLOAT   , OPTIONAL, STATIC  , OTHER, Msg, originatorLng, originatorLat, 0),
    PB_FIELD( 19, INT32   , OPTIONAL, STATIC  , OTHER, Msg, receiverNodeId, originatorLng, 0),
    PB_FIELD( 20, INT32   , OPTIONAL, STATIC  , OTHER, Msg, senderTimeQuality, receiverNodeId, 0),
    PB_FIELD( 21, INT32   , OPTIONAL, STATIC  , OTHER, Msg, originatorEpochTime, senderTimeQuality, 0),
    PB_FIELD( 22, INT32   , OPTIONAL, STATIC  , OTHER, Msg, receiverEpochTime, originatorEpochTime, 0),
    PB_FIELD( 23, INT32   , OPTIONAL, STATIC  , OTHER, Msg, senderEpochTime, receiverEpochTime, 0),
    PB_LAST_FIELD
};


/* @@protoc_insertion_point(eof) */
