#include "SequenceCorner90.h"
#include "CommandDispatcher.h"

// DÉFENSE : "Pourquoi cette séquence appelle-t-elle le dispatcher ?"
// RÉPONSE : "Pour respecter la contrainte de non-duplication. La logique 'Corner 90' étant déjà implémentée et validée dans le Dispatcher, la séquence agit comme une interface standard (HAL) vers ce code existant."

void SequenceCorner90::execute() {
    // On appelle directement la logique centralisée dans le dispatcher
    CommandDispatcher::getInstance().executeCorner90Logic();
}