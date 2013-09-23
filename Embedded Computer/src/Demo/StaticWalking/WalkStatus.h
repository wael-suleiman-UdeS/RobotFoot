/**
 * \file WalkStatus.h
 * \brief Implementation d'un pas sur le robot DarwinOP
 * \author Mathieu Drapeau, Camille Hebert, Antoine Rioux et Maxime Tetrault.
 * \version 0.1
 * \date 27 mars 2013
 *
 * Implementation en c++ d"un pas sur la plateforme DarwinOP a tester
 * durant la validation du cour GEN 744.
 *
 */

#ifndef _WALK_STATUS_H_
#define _WALK_STATUS_H_

#include "Eigen/Dense"
#include <vector>

/*! \class WalkStatus
   * \brief Classe qui calcul le deplacement de la jambe.
   *
   *  Cette classe sera appele dans le framework de DarwinOP
   *  et suit une trajectoire definie.
   */
class WalkStatus
{
public:
  /*!
     *  \brief Constructeur
     *
     *  Constructeur de la classe WalkStatus.
     */
  WalkStatus();
  WalkStatus(const double tf);
  /*!
     *  \brief Destructeur
     *
     *  Destructeur de la classe WalkStatus.
     */
  ~WalkStatus();
  
  /*!
     *  \brief Affiche le tableau DH.
     *
     *  Fonction debug qui permet d'afficher le tableau
     *  DH au moment present.
     */
  void printDH();
  
  /*!
     *  \brief Affiche la matrice Q.
     *
     *  Fonction debug qui permet d'afficher la matrice
     *  Q au moment present (matrice des angles des moteurs).
     */
  void printQ();
  
  /*!
     *  \brief Affiche la matrice D.
     *
     *  Fonction debug qui permet d'afficher la matrice D.
     */
  void printD();
  
  /*!
     *  \brief Affiche la position desire, la position reel,
     *         l'angle desire et l'angle reel.
     *
     *  Fonction debug qui permet d'affiche la position desire,
     *  la position reel, l'angle desire et l'angle reel au moment present.
     */  
  void printPePdTeTd();
  
  /*!
     *  \brief Affiche les parametres de trajectoire.
     *
     *  Fonction debug qui permet d'afficher le tableau
     *  DH au moment present.
     */
  void printTrajParam();  
  
  /*!
     *  \brief Process overload
     *
     *  Apelle Process( const double time,
     *		        const std::vector<double>& pos,
     *		        bool isPosValid );
     *
     *  \param time : Le temps ou le deplacement est rendu.
     *
     */
  void Process( const double time );
  /*!
     *  \brief Iteration de calcul du deplacement de la jambe.
     *
     *  Cette fonction verifie la position des moteurs et determine
     *  la position du pied. Ensuite, on calcul l'erreur entre la 
     *  position desire et la position reel selon le temps ou nous
     *  somme rendu. Meme chose pour l'angle. Ensuite, on calcul la
     *  position a envoyer au moteurs afin de suivre la trajectoire 
     *  et puis on l'envoie.
     *
     *  \param time : Le temps ou le deplacement est rendu.
     *  \param pos : Position des motors lue
     *  \param isPosValid : Use the pos vector only is this is true
     *
     */
  void Process( const double time,
		 const std::vector<double>& pos,
		 bool isPosValid = true );

  /*!
     *  \brief Envoie la commandede position au moteur.
     *     
     *  Envoie la commandede position au moteur en utilisant la 
     *  matrice les mise a jours de la matrice Q.
     * 
     *  Doit etre appeler avant l'execution de l'algorithme pour
     *  placer la robot en position initial.
     */
  void SendMotorValue( const bool rightOnly );
  
  /*!
     *  \brief Renvoie une valeur de position d'un moteur.
     *
     *  Renvoie la valeur de position du moteur correspondant 
     *  a partir de la matrice Q.
     *  
     *  \param ID : Identifiant du moteur.
     *  \return retourne la valeur de position moteur.
     */
  int getMotorValue( const int ID );

   /*!
     *  \brief Initialise la position des moteurs
     *
     *  Initialise les moteurs a leur position de depart par interpolation
     *  
     */
  void initMotorValue();
  
  void getMotorPosition( std::vector<double>& pos );
  
  /*!
     *  \brief Initialise les parametres de la trajectoire cubique du pied
     *
     *  Initialise les parametres de la trajectoire cubique du pied
     *  
     */
  void initAllTrajParam( const double Xf, const double zMax );

private:

  /*!
     *  \brief Retourne une matrice homogene avec les parametres DH.
     *
     *  Genere la matrice homogene entre les deux reperes en parametres.
     *  
     *  \param num1 : Premier repere
     *  \param num2 : Deuxieme repere
     *
     *  \return Matrice homogene
     */
  Eigen::MatrixXd generateMH( const int num1, const int num2 );
  
  /*!
     *  \brief Genere la matrice jacobienne.
     *
     *  Genere la matrice de position et la matrice de rotation de la matrice jacobienne.
     *  
     *  \param J1 : Matrice de position
     *  \param J2 : Matrice de rotation
     */
  void generateJacobian( Eigen::MatrixXd& J1, Eigen::MatrixXd& J2 );  

  /*!
     *  \brief Extraie une colonne d'une matrice vers un vecteur.
     *
     *  Extraie une colonne d'une matrice vers un vecteur.
     *  
     *  \param col : Numero de colonne a extraire (de 0 a size-1)
     *  \param vector : Vecteur de sortie
     *  \param matrix : Matrice d'ou extraire la colonne
     */
  void extractColumnFromMatrix( const int col, Eigen::Vector3d& vector, const Eigen::MatrixXd& matrix );
  
  /*!
     *  \brief Initialise les valeurs des parametres DH
     *
     *  Initialise les valeurs des parametres DH avec les valeurs de depart
     *  
     */
  void initDH();

  /*!
     *  \brief Met a jour les valeurs des parametres DH
     *
     *  Met a jour les valeurs des parametres DH avec les nouvelles valeurs de q
     *  
     */
  void updateDH();
  
  /*!
     *  \brief Initialise les valeurs de q avec les valeurs de depart
     *
     *  Initialise les valeurs de q avec les valeurs de depart
     *  
     */
  void initQ();

  /*!
     *  \brief Initialise les distances entre les joints 
     *
     *   Initialise les distances entre les joints 
     *  
     */
  void initD();

  /*!
     *  \brief Initialise la matrice de rotation
     *
     *  Initialise la matrice de rotation utilisee pour reorienter le pied par     rapport a la hanche
     *  
     */
  void initRot();
  
  /*!
     *  \brief Initialise les positions et orientations initiales
     *
     *  Initialise la position initiale, la position desiree, la rotation initiale
     *  et la rotation desiree du pied.
     *  
     */
  void initPePdTeTd();

  /*!
     *  \brief Actualise la position et la rotation du pied
     *
     *  Actualise la position et la rotation du pied avec la nouvelle matrice
     *  homogene
     *  
     */
  void updatePeTe();

  /*!
     *  \brief Calcule la trajectoire cubique
     *
     *  Calcule la trajectoire cubique du pied avec les parametres de trajectoire
     *  
     */
  void calculateTrajParam( double trajParam[4], double xi, double xf, double tf );
  
  /*!
     *  \brief Met a jour la position desiree
     *
     *  Met a jour la position desiree a l'aide de la trajectoire cubique
     *  
     */
  void UpdatePd( double time );

  /*!
     *  \brief Met a jour la position des moteurs
     *
     *  Met a jour la position des moteurs en les faisant bouger a la position desiree
     *  calculee par la trajectoire cubique
     */
  void UpdateServoPosition();
  
  /*!
     *  \brief Converti un angle de radians vers degres
     *
     *  Converti un angle de radians vers degres
     *  
     *  \param radian : Angle a convertir de radian vers degre
     */
  double RadianToDegree( const double radian );
  
  /*!
     *  \brief Converti un angle de degres vers radians
     *
     *  Converti un angle de degres vers radians
     * 
     *  \param degree : Angle a convertir de degres vers radians 
     */
  double DegreeToRadian( const double degree );
  
    /*!
     *  \brief Enregistre les angles des moteurs dans un fichier texte
     *
     *  Enregistre les angles des moteurs dans un fichier texte en degre
     *  dans un format definie
     * 
     *  \param q : Angles a enregistrer 
     */
  void LogToFile( const std::string fileName, const Eigen::MatrixXd& q );
  
  /*!
     *  \brief Update q matrix with motor position read from external function
     * 
     *  \param pos : Position read from motor
     */
  void UpdateQWithVector( const std::vector<double>& pos );
  
  Eigen::MatrixXd DH;
  Eigen::MatrixXd q;
  Eigen::MatrixXd d;
  Eigen::MatrixXd Rot;
  
  Eigen::Vector3d Pe, Pd, Te, Td;
  
  double xTrajParam[4];
  double yTrajParam[4];
  double zTrajParam1[4];
  double zTrajParam2[4];
  
  double _tf;
};

#endif //_WALK_STATUS_H_
