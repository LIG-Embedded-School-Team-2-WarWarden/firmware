#ifndef ICD_PROTOCOL_H
#define ICD_PROTOCOL_H
#include <cstdint>
#include <string>
#include <vector>
namespace icd {
// Document labels mapped to signed 16-bit wire IDs.
enum MessageId : int16_t { EF01=1, FE01=-1, EF02=2, FE02=-2, EF03=3, FE03=-3, EF04=4, FE04=-4, EF05=5, FE05=-5 };
inline int responseId(int id) {
    switch(id) {
    case EF01: return FE01; case EF02: return FE02; case EF03: return FE03;
    case EF04: return FE04; case EF05: return FE05; default: return 0;
    }
}
enum Result { ACCEPTED=0, BUSY=1, INVALID=2, DEVICE_ERROR=3 };
enum Flag { CONNECTED=1, MOVING=2, LASER=4, DEMO=8, FAULT=16 };
inline unsigned word(const std::string& s, size_t n) {
    return (static_cast<unsigned char>(s.at(n))<<8) | static_cast<unsigned char>(s.at(n+1));
}
inline int signedWord(const std::string& s, size_t n) {
    unsigned v=word(s,n); return v>=32768 ? int(v)-65536 : int(v);
}
inline void append(std::string& s, unsigned v) { s+=char((v>>8)&255); s+=char(v&255); }
inline std::string packet(int id, unsigned tx, unsigned rx, const std::vector<int>& data={}) {
    std::string s; append(s,id); append(s,tx); append(s,rx);
    for (int v:data) append(s,static_cast<unsigned>(v));
    while(s.size()%4) s+='\0';
    return s;
}
inline size_t length(int id) {
    switch(id) {
    case EF01: case FE01: case EF02: case EF04: case FE03: case EF05: case FE05: return 8;
    case FE02: case EF03: case FE04: return 12;
    default: return 0;
    }
}
inline bool validRequest(const std::string& s) {
    if(s.size()<6) return false;
    int id=signedWord(s,0);
    if(responseId(id)==0 || s.size()!=length(id) || word(s,4)!=0) return false;
    size_t pad=id==EF03?10:id==EF04?8:6;
    for(size_t i=pad;i<s.size();++i) if(s[i]!=0) return false;
    return true;
}
}
#endif
