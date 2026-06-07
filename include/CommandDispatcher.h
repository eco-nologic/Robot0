#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include <Arduino.h>

class Robot;

// DÉFENSE : "Quel est l'intérêt du CommandDispatcher ?"
// RÉPONSE : "Il centralise l'interprétation des chaînes de caractères venant du Web ou du BLE. Cela évite la duplication du code de parsing et garantit un comportement identique quel que soit le canal de communication."

class CommandDispatcher {
public:
    /** @brief Accès à l'instance unique. */
    static CommandDispatcher& getInstance();

    /** @brief Lie le dispatcher à l'instance du robot. */
    void begin(Robot& robot);
    /** @brief Interprète et exécute une commande texte. @param cmd Chaîne de commande (ex: "S1", "STOP"). */
    void dispatch(const String& cmd);

    /** @brief Exécute la logique de mouvement du virage Corner 90. */
    void executeCorner90Logic();

    CommandDispatcher(const CommandDispatcher&) = delete;
    CommandDispatcher& operator=(const CommandDispatcher&) = delete;

private:
    CommandDispatcher();
    Robot* _robot;
};

#endif // COMMAND_DISPATCHER_H
