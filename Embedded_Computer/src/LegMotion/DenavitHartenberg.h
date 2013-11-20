#ifndef DENAVIT_HARTENBERG_H
#define DENAVIT_HARTENBERG_H

#include "../../ThirdParty/Eigen/Dense"

class DenavitHartenberg
{
public:

	enum Leg {GroundRight, GroundLeft};
	enum DHSection {ToPelvis, ToFoot};

	DenavitHartenberg(Eigen::VectorXf q, Leg grounedLeg);
	~DenavitHartenberg();

	void Update(Eigen::VectorXf q);
	void UpdateTe(Eigen::VectorXf q);
	Eigen::Matrix4f MatrixHomogene(DHSection section);
	Eigen::Matrix4f MatrixHomogene(Eigen::MatrixXf& DH);
	Eigen::MatrixXf Jacobian(DHSection section, int returnChoice);

	const Eigen::Matrix4f& GetRP1() const { return m_RP_1; }
	const Eigen::Matrix4f& GetPR1() const { return m_PR_1; }
	const Eigen::Matrix4f& GetPR1Fin() const { return m_PR_1_fin; }
	const Eigen::Matrix4f& GetRP2() const { return m_RP_2; }
	const Eigen::Matrix4f& GetPR2Fin() const { return m_PR_2_fin; }
	const Eigen::Vector3f& GetTeToPelvis() const { return m_TeToPelvis; }
	const Eigen::Vector3f& GetTeToFoot() const { return m_TeToFoot; }


	//*****************for test****************************//
	Eigen::MatrixXf positionMatrix;

private:

	void Init(Eigen::VectorXf q);
	Eigen::Matrix4f MatrixHomogene_Vector(Eigen::VectorXf vector);

	Eigen::Matrix4f m_RP_1;
	Eigen::Matrix4f m_PR_1;
	Eigen::Matrix4f m_PR_1_fin;
	Eigen::Matrix4f m_RP_2;
	Eigen::Matrix4f m_PR_2_fin;

	Eigen::MatrixXf m_DHtoPelvis;
	Eigen::MatrixXf m_DHtoFoot;

	Eigen::Vector3f m_TeToPelvis;
	Eigen::Vector3f m_TeToFoot;

	Leg m_groundedFoot;

};

#endif  //DENAVIT_HARTENBERG_H
