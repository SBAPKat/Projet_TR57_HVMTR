#include "PID_Vitesse.h"

// Fonction de calcul du PID de vitesse
PID_Vitesse_resultat PID_Vitesse(PID_Vitesse_resultat resultat, int Erreur, int Erreur_precedent, int Couple_consigne_precedent,int Periode_echantillonnage, int Iq_consigne_precedent){
    resultat = {0};
    resultat.Couple_consigne = Couple_consigne_precedent + KP_Vitesse * (Erreur - Erreur_precedent + (Periode_echantillonnage / Tau_integrale_Vitesse) * Erreur_precedent);
    resultat.Iq_consigne = Iq_consigne_precedent + KP_Vitesse/
    return resultat;
}