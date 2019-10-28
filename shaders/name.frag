#version 450 core

in fData
{
	flat int id;
} In;

layout(location = 0) out float out_id;


void main()
{
	out_id = In.id;
}
