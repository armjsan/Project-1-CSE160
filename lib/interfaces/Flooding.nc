interface Flooding {
    command error_t start();
    command error_t send(pack msg, uint16_t dest);
    command error_t LSAsend(LSA msg, uint8_t totalPayloadSize);
}