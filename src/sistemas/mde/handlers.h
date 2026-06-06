//
// Created by zhl on 6/3/26.
//

#ifndef ACEMA_CTLR_HANDLERS_H
#define ACEMA_CTLR_HANDLERS_H


void handle_mde();
void handle_error();

// --- State Machine Core Execution ---
void runStateMachine();
// --- Transition / Guard Conditions Logic ---
void handleStateTransitions();

#endif //ACEMA_CTLR_HANDLERS_H
