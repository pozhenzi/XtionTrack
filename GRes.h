
struct PosPoint
{
	float X;
	float Y;
	float Z;
};

struct JumpPosition
{
	PosPoint Neck;
	PosPoint Left_Shoulder;
	PosPoint Right_Shoulder;
	PosPoint Tosor;
};
struct LeanPosition
{
	PosPoint Head;
	PosPoint Neck;
	PosPoint Tosor;
};
enum PositiondirectionEnum
{
	T0_THE_LEFT_OF = 1,
	TO_THE_RIGHT_OF = 2,
	IN_FRONT_OF = 3,
	BEHIND=4,
	ABOVE=5,
	BELOW=6,
	APART_FROM=7,
};
enum BodyEnum
{
	LEAN_LEFT = 1,
	LEAN_RIGHT = 2,
	TURN_LEFT =3,
	TURN_RIGHT=4,
	JUMP=5,
};
enum velocityDirection
{
	T0_THE_LEFT = 0,
	TO_THE_RIGHT=1,
	FORWARD=2,
	BACKWARD=3,
	UP=4,
	DOWN=5,
	IN_ANY_DIRECTION=6,
};