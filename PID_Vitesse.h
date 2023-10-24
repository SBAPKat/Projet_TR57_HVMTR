
#define KP_Vitesse 1
#define KI_Vitesse 1
#define Tau_integrale_Vitesse 1



typedef struct{
    int Couple_consigne;
    int Iq_consigne;
} PID_Vitesse_resultat;

PID_Vitesse_resultat PID_Vitesse(PID_Vitesse_resultat resultat, int Erreur, int Erreur_precedent, int Couple_consigne_precedent,int Periode_echantillonnage, int Iq_consigne_precedent);
