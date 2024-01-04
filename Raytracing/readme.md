- Cell Shading 
    - 
    - (--shader toon)
    - L'implementazione è composta da 4 fasi:
        1. cerco la fonte di luce (ovvero il materiale nella scena che ha l'emission diversa da 0) e prendo l'origine deli'istanza dell'oggetto come punto dalla quale viene emessa.
        2. controllo se il punto di intersezione del raggio attualmente preso in considerazione è o meno colpito direttamente dalla luce. Per fare ciò, faccio partire un raggio da quel punto verso la luce e se prima di arrivarvi colpisce un oggetto (diverso da quello che ha emission diversa da 0), allora lo considero come ombra.
    
    <img src="/home/jbox/remnote/alessandrofantesini@gmail.com/files/oLlWifFR_j-gHzmWarmJ4rhXssYo4vynDmeVEePthCseP0DfzILf0x3SIyo2Y73oAMAB7VarOlId519I62jF7bjVAoiWV6oeiOjDOqinj_0fnWhrGNbspaeI_Ua2oXb1.png" alt="oLlWifFR_j-gHzmWarmJ4rhXssYo4vynDmeVEePthCseP0DfzILf0x3SIyo2Y73oAMAB7VarOlId519I62jF7bjVAoiWV6oeiOjDOqinj_0fnWhrGNbspaeI_Ua2oXb1" width="100"/> 

    3. calcola l'intensità della luce con il prodotto tra vettore della normale e vettore della luce
    4. aggiungo luminosità al punto in caso sia speculare al punto luce o sia l'anello esterno
    
    <img src="/home/jbox/remnote/alessandrofantesini@gmail.com/files/NJ40ceVPBrGQ-aSKlvw3E35h5ykGSOF45rn69gEDJKIDXL_PpP4ZW9MH7V9LHPjZtpAVAt0lfhO1paBxPD0C_MRi4GFpvm7e9CG9urdZO8X62nlHveUxx4nSN4a_cKFh.png" alt="NJ40ceVPBrGQ-aSKlvw3E35h5ykGSOF45rn69gEDJKIDXL_PpP4ZW9MH7V9LHPjZtpAVAt0lfhO1paBxPD0C_MRi4GFpvm7e9CG9urdZO8X62nlHveUxx4nSN4a_cKFh" width="80"/>  

    - Il risultato finale
    - ![](/home/jbox/remnote/alessandrofantesini@gmail.com/files/MSm2__pJvzC62qGuIMlCL2bR8-UW_4n3nB3onmUpgaXexLh5hPZAE4wHtVV8pTPdrLd918KZEg0IVmdnBET1T2DO-2pIkjRBM2tnrMa3aje_5rydm3inPdFlphg3e0Cy.jpeg) 
    - ![](/home/jbox/remnote/alessandrofantesini@gmail.com/files/SdXEtCTIXFQyUn_EigpbLqhh1VBy_vcL2f2IIaSUlYMTArKDmZq-Qsu-Agszd-Wg_-5shDYu9OkJrg7VgMf0HkB6BtknU7zTTHWXyhtbKmGp2Id0bCXw68WVbBy6b0sE.jpeg) 
    - ![](/home/jbox/remnote/alessandrofantesini@gmail.com/files/sQ-Bo8XSTKnG3bez_6C1BEF8sNapg0LcMdawL7rt8nXcCkbtNrSO2SJsF6JYb94GrHg8_2a-9LbOqsp6GYYSSrT9X935qTWPxBDCP3uUZrJJzyVW9te68lS5k6kOhgBc.jpeg) 