// #include <stdint.h>
// #include <stdbool.h>
// #include <stdio.h>
// #include <string.h>

// int cur_seq = -1;
// unsigned int window = 3;
// void int2bin(uint8_t c, int* out) {
//     for(int i = 0; i < 8; i++) {
// 		out[7 - i] = (int)(c & 0x01);
// 		c = c >> 1;
// 	}
// }

// void crc3(int* in, int* out) {
//     int tmp[41];
//     int crc3[4] = {1, 0, 1, 1};
//     for (int i = 0; i < 41; i++) {
//         if (i < 38) {
//             tmp[i] = in[i];
//         } else {
//             tmp[i] = 0;
//         }
//     }
//     for (int i = 0; i < 38; i++) {
//         if (tmp[i] == 0) {
//             continue;
//         }
//         for (int j = 0; j < 4; j++) {
//             tmp[i + j] = tmp[i + j] ^ crc3[j];
//         }
//     }
//     for (int i = 38; i < 41; i++) {
//         out[i - 38] = tmp[i];
//     }
// }

// bool extract_payload(uint8_t * ints, char * chars) {
//     int bits[48];
//     int c[8];
//     for (int i = 0; i < 6; i++) {
//         uint8_t tmp = ints[i];
//         int2bin(tmp, c);
//         for (int j = 0; j < 8; j++) {
//             bits[8*i+j] = c[j];
//         }
//     }

//     int size_and_parity[7] = {1,0,1,0,0,1,1};
//     if (memcmp(size_and_parity, bits, sizeof(size_and_parity)) != 0) {
//         return false;
//     }

//     int crc_calc[38];
//     int crc_check[3];
//     memcpy(crc_calc, bits + 7, sizeof(crc_calc));
//     crc3(crc_calc, crc_check);
//     for (int i = 0; i < 3; i++) {
//         if (bits[45 + i] != crc_check[i]) {
//             return false;
//         }
//     }

//     int src_node[8] = {0,0,0,0,0,0,0,1};
//     if (memcmp(src_node, bits + 7, sizeof(src_node)) != 0) {
//         return false;
//     }

//     int dest_node[8] = {0,0,0,0,0,1,0,1};
//     if (memcmp(dest_node, bits + 15, sizeof(dest_node)) != 0) {
//         return false;
//     }

//     int new_seq = 0;
//     for (int i = 23; i < 29; i++) {
//         new_seq = 2 * new_seq + bits[i];
//     }
//     if ((new_seq == 0) && (cur_seq == -1)) {
//         cur_seq = new_seq;
//     } else if ((new_seq - cur_seq > 0) && (new_seq - cur_seq < window)) {
//         cur_seq = new_seq;
//     } else if ((new_seq + (64 - cur_seq)) > 0 && (new_seq + (64 - cur_seq)) < window) {
//         cur_seq = new_seq;
//     } else {
//         return false;
//     }

//     uint8_t char1 = 0;
//     for (int i = 29; i < 37; i++) {
//         char1 = 2 * char1 + bits[i];
//     }
//     chars[0] = (char) char1;

//     uint8_t char2 = 0;
//     for (int i = 37; i < 45; i++) {
//         char2 = 2 * char2 + bits[i];
//     }
//     chars[1] = (char) char2;

    
    
//     return true;
// }

// int main() {
//     char chars[2];
//     uint8_t all_bits0[6] = {166, 2, 10, 3, 67, 40};
//     bool ok = extract_payload(all_bits0, chars);
//     if (ok) {
//         printf("%c", chars[0]);
//         printf("%c\n", chars[1]);
//     }
    
//     uint8_t all_bits1[6] = {166, 2, 10, 11, 99, 100};
//     ok = extract_payload(all_bits1, chars);
//     if (ok) {
//         printf("%c", chars[0]);
//         printf("%c\n", chars[1]);
//     }
//     uint8_t all_bits2[6] = {166, 2, 10, 19, 120, 3};
//     ok = extract_payload(all_bits2, chars);
//     if (ok) {
//         printf("%c", chars[0]);
//         printf("%c\n", chars[1]);
//     }
// }