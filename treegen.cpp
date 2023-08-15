/*=====================================================================
treegen.cpp
-----------
Copyright Nicholas Chapman 2023 -
=====================================================================*/


#include <graphics/PNGDecoder.h>
#include <graphics/ImageMap.h>
#include <maths/GeometrySampling.h>
#include <dll/include/IndigoMesh.h>
#include <graphics/PerlinNoise.h>
#include <opengl/OpenGLShader.h>
#include <opengl/OpenGLProgram.h>
#include <opengl/OpenGLEngine.h>
#include <opengl/GLMeshBuilding.h>
#include <indigo/TextureServer.h>
#include <maths/PCG32.h>
#include <opengl/VBO.h>
#include <opengl/VAO.h>
#include <utils/Exception.h>
#include <utils/StandardPrintOutput.h>
#include <utils/IncludeWindows.h>
#include <utils/PlatformUtils.h>
#include <utils/ConPrint.h>
#include <utils/StringUtils.h>
#include <GL/gl3w.h>
#include <SDL_opengl.h>
#include <SDL.h>
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl.h>
#include <iostream>
#include <fstream>
#include <string>


struct TreeParams
{
	int max_depth;
	float trunk_base_radius;
	float radius_falloff_factor;
	float trunk_base_len;
	float branch_len_depth_factor;
	float trunk_upper_radius;
	float trunk_noise_freq;
	float trunk_noise_mag;
	float trunk_bump_freq;
	float trunk_bump_mag;
	float tension;
	float phototropism_constant;
	float phototropism_depth_factor;
	float gravity_bend;

	float leaf_width;
	float leaf_height;
};

// https://en.wikipedia.org/wiki/B%C3%A9zier_curve, Cubic Bézier curves
//Vec4f bezierCurve(const Vec4f& p0, const Vec4f& p1, const Vec4f& p2, const Vec4f& p3, float t)
//{
//	return p0 * Maths::pow3(1 - t) + p1*(3*Maths::square(1 - t)*t) + p2 * (3*(1-t)*Maths::square(t)) + p3 * Maths::pow3(t);
//}

struct LimbVert
{
	GLARE_ALIGNED_16_NEW_DELETE

	LimbVert() {}
	LimbVert(Vec4f pos_, float radius_) : pos(pos_), radius(radius_) {}

	Vec4f pos;
	float radius;
};


void appendCylinder(const TreeParams& params, Indigo::Mesh& mesh, std::vector<LimbVert>& path_verts, /*float radius_a, float radius_b, */int vert_num_segs, int around_num_segs, float path_noise_freq, float path_noise_mag,
	float bump_noise_freq, float bump_noise_mag)
{
	if(path_verts.size() < 2)
		return;

	// Create larger array of path points with even spacing
	
	Matrix4f basis_m;
	basis_m.constructFromVector(normalise(path_verts[1].pos - path_verts[0].pos));
	Vec4f basis_i = basis_m.getColumn(0);
	Vec4f basis_j = basis_m.getColumn(1);

	mesh.setMaxNumTexcoordSets(1);
	mesh.num_materials_referenced = 2;

	const int bump_period = myClamp<int>((int)Maths::roundToNextHighestPowerOf2((int)(pow(2, bump_noise_freq * 8.f))), 1, 256);
	//printVar(bump_period);

	const uint32 vert_offset = (uint32)mesh.vert_positions.size();

	// Add verts

	vert_num_segs = 30;
	
	//for(int y=0; y<vert_num_segs; ++y)
	Vec4f prev_path_centre = path_verts[0].pos;
	const int total_vert_segs = ((int)path_verts.size() - 1) * vert_num_segs;
	{
	int y = 0;
	for(int start_i = 0; start_i < (int)path_verts.size() - 1; start_i++)
	{
		const int end_i = start_i + 1;

		/*Vec4f p0;
		if(start_i == 0)
			p0 = path_verts[0] + (path_verts[0] - path_verts[1]);
		else
			p0 = path_verts[start_i - 1];
		Vec4f p1 = path_verts[start_i];
		Vec4f p2 = path_verts[end_i];
		Vec4f p3;
		if(end_i + 1 >= (int)path_verts.size())
			p3 = path_verts[end_i] + (path_verts[end_i] - path_verts[end_i-1]);
		else
			p3 = path_verts[end_i + 1];*/

		/*Vec4f prev_vert;
		if(start_i == 0)
			prev_vert = path_verts[0] + (path_verts[0] - path_verts[1]);
		else
			prev_vert = path_verts[start_i - 1];

		Vec4f next_vert;
		if(end_i + 1 >= (int)path_verts.size())
			next_vert = path_verts[end_i] + (path_verts[end_i] - path_verts[end_i-1]);
		else
			next_vert = path_verts[end_i + 1];

		const float tension = params.tension;
		Vec4f p0 = path_verts[start_i];
		Vec4f p1 = p0 + normalise(p0 - prev_vert) * tension;
		Vec4f p2 = path_verts[end_i] + normalise(path_verts[end_i] - next_vert) * tension;
		Vec4f p3 = path_verts[end_i];*/

		Vec4f prev_vert;
		if(start_i == 0)
			prev_vert = path_verts[0].pos + (path_verts[0].pos - path_verts[1].pos);
		else
			prev_vert = path_verts[start_i - 1].pos;

		Vec4f next_vert;
		if(end_i + 1 >= (int)path_verts.size())
			next_vert = path_verts[end_i].pos + (path_verts[end_i].pos - path_verts[end_i-1].pos);
		else
			next_vert = path_verts[end_i + 1].pos;

		/*Vec4f t0 = path_verts[start_i] - prev_vert;
		Vec4f t1 = next_vert - path_verts[end_i];*/
		Vec4f p0 = path_verts[start_i].pos;
		Vec4f p1 = path_verts[end_i].pos;
		Vec4f t0 = (p1 - prev_vert) * params.tension; // Tangent vector at vert 0
		Vec4f t1 = (next_vert - p0) * params.tension;// Tangent vector at vert 1
		

		//const float tension = params.tension;
		//Vec4f p0 = path_verts[start_i];
		//Vec4f p1 = p0 + normalise(p0 - prev_vert) * tension;
		//Vec4f p2 = path_verts[end_i] + normalise(path_verts[end_i] - next_vert) * tension;
		//Vec4f p3 = path_verts[end_i];


		for(int q=0; q<vert_num_segs; ++q)
		{
			const float vert_frac = (float)y / total_vert_segs;
			


			/*
			path_verts.size() == 5

			|------------|-------------|-------------|------------|
			0            1            2             3            4
			*/

			//int start_i = myMin((int)path_verts.size() - 1, (int)(vert_frac * (path_verts.size() - 1)));
			//int end_i   = myMin((int)path_verts.size() - 1, (int)ceil(vert_frac * path_verts.size()));

			assert(start_i != end_i);
			float seg_frac = (float)q / vert_num_segs;//vert_frac * path_verts.size() - (float)start_i;

			//const Vec4f seg_dir = normalise(path_verts[end_i] - path_verts[start_i]);

			const Vec4f q0 = p0 + t0 * seg_frac;
			const Vec4f q1 = p1 - t1 * (1 - seg_frac);

			const Vec4f path_centre = Maths::lerp(q0, q1, Maths::smoothStep(0.f, 1.f, seg_frac));

			//const Vec4f path_centre = bezierCurve(p0, p1, p2, p3, seg_frac);

			// Adjust basis so it's still orthogonal to segment dir
			if(y > 0)
			{
				const Vec4f seg_dir = normalise(path_centre - prev_path_centre);
				basis_i = normalise(removeComponentInDir(basis_i, seg_dir));
				basis_j = normalise(removeComponentInDir(basis_j, seg_dir));
			}

			//Vec4f pre_noise_centre = Maths::lerp(path_verts[start_i], path_verts[end_i], seg_frac);
			
			

			//const float r = Maths::lerp(radius_a, radius_b, vert_frac);
			//const float r = radius_a * exp(-vert_frac * path_verts.size() * 0.3f);
			//const float r = Maths::lerp(path_verts[0].radius, path_verts.back().radius, vert_frac);
			const float r = Maths::lerp(path_verts[start_i].radius, path_verts[end_i].radius, seg_frac);

			for(int x=0; x<around_num_segs; ++x)
			{
				const float xfrac = (float)x / (around_num_segs - 1);
				const float phi = xfrac * Maths::get2Pi<float>();

				//Vec4f pre_noise_centre = end_a + (end_b - end_a) * vert_frac;
				Vec4f noise = PerlinNoise::FBM4Valued(path_centre * path_noise_freq, 5) * path_noise_mag;
				//pre_noise_centre += m.getColumn(0) * noise[0];
				//pre_noise_centre += m.getColumn(1) * noise[1];
				Vec4f post_noise_centre = path_centre + noise;

			
				float bumped_r = r * (1.f + PerlinNoise::periodicFBM(xfrac * bump_period, vert_frac * bump_period * 3.14f, /*num_octaves=*/5, /*period=*/bump_period) * bump_noise_mag);

				const Vec4f pos = post_noise_centre +
					basis_i * cos(phi) * bumped_r + 
					basis_j * sin(phi) * bumped_r;
				
				mesh.vert_positions.push_back(Indigo::Vec3f(pos[0], pos[1], pos[2]));

				mesh.uv_pairs.push_back(Indigo::Vec2f(xfrac, vert_frac * path_verts.size()));
			}

			prev_path_centre = path_centre;
			y++;
		}
	}
	}

	//const int final_num_segs = total_vert_segs;
	for(uint32 y=1; y<(uint32)total_vert_segs; ++y)
	{
		for(uint32 x=1; x<(uint32)around_num_segs; ++x)
		{
			const uint32 x_minus_1 = x - 1;
			//const uint32 x_minus_1 = (x == 0) ? (around_num_segs - 1) : x - 1;
			//const uint32 y_minus_1 = (y == 0) ? (around_num_segs - 1) : x - 1;

			uint32 vert_indices_1[3] = { vert_offset + (y-1) * around_num_segs + x_minus_1, vert_offset + (y-1) * around_num_segs + x, vert_offset + y * around_num_segs + x };
			mesh.addTriangle(vert_indices_1, vert_indices_1, /*material_index=*/0);

			uint32 vert_indices_2[3] = { vert_offset + (y-1) * around_num_segs + x_minus_1, vert_offset + y * around_num_segs + x , vert_offset + y * around_num_segs + x_minus_1 };
			mesh.addTriangle(vert_indices_2, vert_indices_2, /*material_index=*/0);
		}
	}
}


struct Leaf
{
	Vec4f pos; // Point of attachment to branch

	Vec4f spine_dir;
	Vec4f face_normal;
};

struct Limb : public RefCounted
{
	GLARE_ALIGNED_16_NEW_DELETE

	Vec4f startpos, endpos;
	float start_radius, end_radius;

	std::vector<Reference<Limb>> children; // main child
	//Reference<Limb> child_b; // side child

	std::vector<Leaf> leaves;
};

typedef Reference<Limb> LimbRef;

inline Indigo::Vec3f toIndigoVec3f(const Vec4f& v)
{
	return Indigo::Vec3f(v[0], v[1], v[2]);
}

void makeSubTreeMesh(const TreeParams& params, Indigo::Mesh& mesh, LimbRef limb, PCG32& rng, int depth, bool child_a_path_created_for_subtree)
{
	// pick some direction to go before making a side branch
	//const float branch_dist = rng.unitRandom();

	//const float radius_at_branch = branch_radius * 0.6f;


	// Make any leaf quads
	for(size_t i=0; i<limb->leaves.size(); ++i)
	{
		const Leaf& leaf = limb->leaves[i];

		const uint32 vert_offset = (uint32)mesh.vert_positions.size();

		const Vec4f leaf_right = crossProduct(leaf.spine_dir, leaf.face_normal);
		Vec4f v0 = leaf.pos - leaf_right * params.leaf_width*0.5f;
		Vec4f v1 = leaf.pos + leaf_right * params.leaf_width*0.5f;
		Vec4f v2 = leaf.pos + leaf_right * params.leaf_width*0.5f + leaf.spine_dir * params.leaf_height;
		Vec4f v3 = leaf.pos - leaf_right * params.leaf_width*0.5f + leaf.spine_dir * params.leaf_height;

		mesh.vert_positions.push_back(toIndigoVec3f(v0));
		mesh.uv_pairs.push_back(Indigo::Vec2f(0, 0));

		mesh.vert_positions.push_back(toIndigoVec3f(v1));
		mesh.uv_pairs.push_back(Indigo::Vec2f(1, 0));

		mesh.vert_positions.push_back(toIndigoVec3f(v2));
		mesh.uv_pairs.push_back(Indigo::Vec2f(1, 1));

		mesh.vert_positions.push_back(toIndigoVec3f(v3));
		mesh.uv_pairs.push_back(Indigo::Vec2f(0, 1));

		const uint32 vert_indices_1[3] = { vert_offset + 0, vert_offset + 1, vert_offset + 2 };
		const uint32 vert_indices_2[3] = { vert_offset + 0, vert_offset + 2, vert_offset + 3 };
		mesh.addTriangle(vert_indices_1, vert_indices_1, /*material_index=*/1);
		mesh.addTriangle(vert_indices_2, vert_indices_2, /*material_index=*/1);
	}

	if(!child_a_path_created_for_subtree)
	{
		// Get the list of verts along child_a path
		std::vector<LimbVert> child_a_path_verts;
		child_a_path_verts.push_back(LimbVert(limb->startpos, limb->start_radius));

		LimbRef cur_limb = limb;
		while(cur_limb.nonNull())
		{
			child_a_path_verts.push_back(LimbVert(cur_limb->endpos, cur_limb->end_radius));

			if(!cur_limb->children.empty())
				cur_limb = cur_limb->children[0];
			else
				cur_limb = NULL;
		}


		const int res = 20;// / (depth + 1);

		appendCylinder(params, mesh, child_a_path_verts, res, res, params.trunk_noise_freq, params.trunk_noise_mag,
			params.trunk_bump_freq, params.trunk_bump_mag);
	}

	// split into two branches
	
	for(size_t i=0; i<limb->children.size(); ++i)
		makeSubTreeMesh(params, mesh, limb->children[i], rng, depth + 1, /*child_a_path_created_for_subtree=*/i == 0);

	//if(root_limb->child_a.nonNull())
	//	makeSubTreeMesh(params, mesh, root_limb->child_a, rng, depth + 1, /*child_a_path_created_for_subtree=*/true);
	//if(root_limb->child_b.nonNull())
	//	makeSubTreeMesh(params, mesh, root_limb->child_b, rng, depth + 1, /*child_a_path_created_for_subtree=*/false);
}


LimbRef makeLimbSubTree(const TreeParams& params, const Vec4f& branch_start, const Vec4f& initial_branch_dir, PCG32& rng, float branch_radius, int depth, float dist_since_branch, 
	float total_limb_len) // len till termination or branch
{
	//const float branch_dist = 1.f * exp(-depth * params.branch_len_depth_factor);//  0.3f + 0.3f * rng.unitRandom();

	bool branch = false;
	float limb_len = 0.3f;
	if(limb_len + dist_since_branch > total_limb_len)
	{
		limb_len = myMax(0.15f, total_limb_len - dist_since_branch); // TEMP: too short limb screws up b-spline, fix
		branch = true;
	}


	Vec4f new_branch_dir = initial_branch_dir;
	// Do rotation up/down
	{
		if(absDot(initial_branch_dir, Vec4f(0,0,1,0)) < 0.999f)
		{
			const float cur_theta = std::acos(dot(initial_branch_dir, Vec4f(0,0,1,0)));
			Vec4f rot_axis = normalise(crossProduct(initial_branch_dir, Vec4f(0,0,1,0)));


			//const float phototropism_depth_factor = params.phototropism_constant + params.phototropism_depth_factor * depth;//exp(depth);

			const float phototropism_rot_angle = params.phototropism_constant + (float)depth * params.phototropism_depth_factor;//myMin(params.phototropism, cur_theta);
			

			// Gravity bend: bend downwards, proportional to how sideways this branch is (and its radius?)
			const float sideways = 1 - absDot(initial_branch_dir, Vec4f(0,0,1,0));
			const float gravity_rot_angle = -sideways * params.gravity_bend;

			const float raw_rot = phototropism_rot_angle + gravity_rot_angle;
			const float rot_angle = myClamp(raw_rot, -Maths::pi<float>() + cur_theta, cur_theta); // Don't rotate past pointing all the way up, or pointing all the way down.
			// Not rotating past all the way up implies rot should be <= theta

			new_branch_dir = Matrix4f::rotationMatrix(rot_axis, rot_angle) * initial_branch_dir;

		}
	}

	


	//const float radius_at_limb_end = myMax(0.001f, branch_radius - limb_len * 0.02f);//  exp(-limb_len * 0.9f);
	const float radius_at_limb_end = branch_radius * exp(-limb_len * params.radius_falloff_factor);

	LimbRef limb = new Limb();
	limb->startpos = branch_start;
	limb->endpos = branch_start + new_branch_dir * limb_len;
	limb->start_radius = branch_radius;
	limb->end_radius = radius_at_limb_end;


	if(branch_radius < 0.002f) // depth == params.max_depth)
	{
		// Make leaves
		const float leaf_spacing = 0.05f;
		const int num_leaves = (int)(limb_len / leaf_spacing);

		Vec4f right_vec;
		if(abs(new_branch_dir[2]) > 0.999f)
			right_vec = Vec4f(1,0,0,0);
		else
			right_vec = normalise(crossProduct(new_branch_dir, Vec4f(0,0,1,0)));

		float leaf_angle = 0; // angle to right_vec in right_vec-up plane
		for(int z=0; z<num_leaves; ++z)
		{
			const Vec4f leaf_pos = branch_start + new_branch_dir * (float)z * leaf_spacing;

			float use_angle;
			if(cos(leaf_angle) > 0)
				use_angle = leaf_angle - 0.3f;
			else
				use_angle = leaf_angle + 0.3f;
			Leaf leaf;
			leaf.pos = leaf_pos;
			leaf.spine_dir = right_vec * cos(use_angle) + Vec4f(0,0,1,0)*sin(use_angle);
			leaf.face_normal = normalise(crossProduct(leaf.spine_dir, new_branch_dir));
			if(leaf.face_normal[2] < 0)
				leaf.face_normal *= -1.f; // make sure faces upwards

			limb->leaves.push_back(leaf);

			leaf_angle += Maths::pi<float>();
		}
	}

	if(branch)
	{
		if(depth < params.max_depth && radius_at_limb_end >= 0.001)
		{
			const int num_branches = 4;

			Matrix4f basis;
			basis.constructFromVector(new_branch_dir);

			const float branch_0_angle = rng.unitRandom() * Maths::get2Pi<float>();

			std::vector<float> weights(num_branches);
			float weightsum = 0;
			for(int i=0; i<num_branches; ++i)
			{
				weights[i] = 0.1f + rng.unitRandom();
				weightsum += weights[i];
			}

			// move greatest weight to weights[0];
			std::sort(weights.begin(), weights.end());
			std::reverse(weights.begin(), weights.end());

			// normalise
			for(int i=0; i<num_branches; ++i)
				weights[i] /= weightsum;

			const float branch_A = Maths::pi<float>() * Maths::square(radius_at_limb_end); // Cross-sectional area at branch

			for(int i=0; i<num_branches; ++i)
			{
				const float branch_angle = branch_0_angle + i * (Maths::get2Pi<float>() / num_branches);
				Vec4f orthogonal_branch_dir = basis.getColumn(0) * cos(branch_angle) + basis.getColumn(1) * sin(branch_angle);

				const float angle = myMin<float>(Maths::pi_2<float>(), 0.3f / (weights[i] + 0.3f)); // Angle from new_branch_dir

				const float branch_a_A = branch_A * weights[i];
				const float branch_a_r = myMin(radius_at_limb_end, std::sqrt(branch_a_A * 1.0f * Maths::recipPi<float>()));

				Vec4f branch_a_dir = new_branch_dir * cos(angle) + orthogonal_branch_dir * sin(angle);

				const float child_total_limb_len = total_limb_len * (0.4f + rng.unitRandom() * 0.6f) * exp(-(depth + 1) * params.branch_len_depth_factor);
				limb->children.push_back(makeLimbSubTree(params, /*branch_start=*/limb->endpos, /*branch_dir=*/branch_a_dir, rng, branch_a_r, depth + 1, /*dist_since_branch=*/0, child_total_limb_len));
			}



#if 0
			// Pick random direction orthogonal to branch_dir to split around
			Vec4f split_axis = normalise(::removeComponentInDir(Vec4f(-1 + rng.unitRandom() * 2, -1 + rng.unitRandom() * 2, -1 + rng.unitRandom() * 2, 0), new_branch_dir));

			const float a_weight = 0.1f + rng.unitRandom() * 0.8f;
			const float b_weight = 1 - a_weight;

			const float a_angle = 0.3f / (a_weight + 0.2f);
			const float b_angle = 0.3f / (b_weight + 0.2f);
			Vec4f a_dir = Matrix4f::rotationMatrix(split_axis,  a_angle) * new_branch_dir;
			Vec4f b_dir = Matrix4f::rotationMatrix(split_axis, -b_angle) * new_branch_dir;

			// A = pi r^2
			// r^2 = A/pi
			// r = sqrt(A/pi)
			const float branch_A = Maths::pi<float>() * Maths::square(radius_at_limb_end); // Cross-sectional area at branch

			const float branch_a_A = branch_A * a_weight;
			const float branch_b_A = branch_A * b_weight;

			const float branch_a_r = myMin(radius_at_limb_end, std::sqrt(branch_a_A * 1.2f * Maths::recipPi<float>()));
			const float branch_b_r = myMin(radius_at_limb_end, std::sqrt(branch_b_A * 1.2f * Maths::recipPi<float>()));
	
			limb->child_a = makeLimbSubTree(params, /*branch_start=*/limb->endpos, /*branch_dir=*/a_dir, rng, branch_a_r, depth + 1, /*dist_since_branch=*/0);
			limb->child_b = makeLimbSubTree(params, /*branch_start=*/limb->endpos, /*branch_dir=*/b_dir, rng, branch_b_r, depth + 1, /*dist_since_branch=*/0);
#endif
		}


		// Else branch is terminated.

		

	}
	else
	{
		// Don't branch, just continue limb
		limb->children.push_back(makeLimbSubTree(params, /*branch_start=*/limb->endpos, /*branch_dir=*/new_branch_dir, rng, /*branch_radius=*/radius_at_limb_end, depth, dist_since_branch + limb_len, total_limb_len));
	}
	
	return limb;
}


Indigo::MeshRef makeTreeMesh(const TreeParams& params)
{
	Indigo::MeshRef mesh = new Indigo::Mesh();

	PCG32 rng(1);

	// Make limb hierarchy
	LimbRef root_limb = makeLimbSubTree(params, Vec4f(0,0,0,1), Vec4f(0,0,1,0), rng, params.trunk_base_radius, /*depth=*/0, /*dist_since_branch=*/0.f, /*total_limb_len=*/params.trunk_base_len);

	makeSubTreeMesh(params, *mesh, root_limb, rng, /*depth=*/0, /*child_a_path_created_for_subtree=*/false);
	//appendCylinder(*mesh, Vec4f(0,0,0,1), Vec4f(0,0,2,1), params.trunk_base_radius, params.trunk_upper_radius, 200, 200, params.trunk_noise_freq, params.trunk_noise_mag,
	//	params.trunk_bump_freq, params.trunk_bump_mag);

	// Append side branch


	return mesh;
}


GLObjectRef makeTreeObject(const std::string& base_dir, const TreeParams& tree_params, OpenGLEngine& opengl_engine)
{
	Indigo::MeshRef tree_mesh = makeTreeMesh(tree_params);

	GLObjectRef tree_ob = new GLObject();
	tree_ob->mesh_data =  GLMeshBuilding::buildIndigoMesh(opengl_engine.vert_buf_allocator.ptr(), tree_mesh, /*skip_opengl_calls=*/false);
	tree_ob->ob_to_world_matrix = Matrix4f::uniformScaleMatrix(1);
	tree_ob->materials.resize(2);
	//tree_ob->materials[0].albedo_texture = opengl_engine.getTexture(base_dir + "/resources/obstacle.png");
	tree_ob->materials[0].albedo_texture = opengl_engine.getTexture(base_dir + "/resources/bark04.png");
	tree_ob->materials[0].roughness = 0;
	//tree_ob->materials[0].tex_matrix = Matrix2f(10.f, 0, 0, 10.f);

	tree_ob->materials[1].albedo_texture = opengl_engine.getTexture(base_dir + "/resources/dry_leaf42_diff.png");
	tree_ob->materials[1].roughness = 0.6f;
	tree_ob->materials[1].tex_matrix = Matrix2f(1, 0, 0, -1);

	return tree_ob;
}


void setGLAttribute(SDL_GLattr attr, int value)
{
	const int result = SDL_GL_SetAttribute(attr, value);
	if(result != 0)
	{
		const char* err = SDL_GetError();
		throw glare::Exception("Failed to set OpenGL attribute: " + (err ? std::string(err) : "[Unknown]"));
	}
}


int main(int, char**)
{
	Clock::init();

	//========================== Tests ============================
	if(false)
	{
#if BUILD_TESTS
		return 0;
#endif
	}
	//========================== End Tests ============================

	try
	{
		int primary_W = 1680;
		int primary_H = 1000;
		uint32 window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;

		conPrint("Using primary window resolution   " + toString(primary_W) + " x " + toString(primary_H));

		SDL_Window* win = SDL_CreateWindow("TreeGen", 100, 100, 1920, 1080, window_flags | SDL_WINDOW_RESIZABLE);
		if(win == nullptr)
			throw glare::Exception("SDL_CreateWindow Error: " + std::string(SDL_GetError()));

		//setGLAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
		//setGLAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
		setGLAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

		setGLAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

		SDL_GLContext gl_context = SDL_GL_CreateContext(win);
		if(!gl_context)
			throw glare::Exception("OpenGL context could not be created! SDL Error: " + std::string(SDL_GetError()));

		if(SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1) != 0)
			throw glare::Exception("SDL_GL_SetAttribute Error: " + std::string(SDL_GetError()));


		gl3wInit();


		// Initialise ImGUI
		ImGui::CreateContext();

		ImGui_ImplSDL2_InitForOpenGL(win, gl_context);
		ImGui_ImplOpenGL3_Init();


		// Create OpenGL engine
		OpenGLEngineSettings settings;
		settings.compress_textures = true;
		settings.shadow_mapping = true;
		Reference<OpenGLEngine> opengl_engine = new OpenGLEngine(settings);
		opengl_engine->are_8bit_textures_sRGB = true;

		TextureServer* texture_server = new TextureServer(/*use_canonical_path_keys=*/false);

		const std::string base_src_dir(BASE_SOURCE_DIR);
		//const std::string indigo_trunk_dir(INDIGO_TRUNK);

		const std::string data_dir = "N:\\glare-core\\trunk/opengl";
		StandardPrintOutput print_output;
		opengl_engine->initialise(data_dir, texture_server, &print_output);
		opengl_engine->setViewport(primary_W, primary_H);


		const std::string base_dir = ".";//base_src_dir;


		opengl_engine->setMainViewport(primary_W, primary_H);

		std::cout << "Finished compiling and linking program." << std::endl;



		const float sun_phi = 1.f;
		const float sun_theta = Maths::pi<float>() / 4;
		opengl_engine->setSunDir(normalise(Vec4f(std::cos(sun_phi) * sin(sun_theta), std::sin(sun_phi) * sin(sun_theta), cos(sun_theta), 0)));
		opengl_engine->setEnvMapTransform(Matrix3f::rotationMatrix(Vec3f(0,0,1), sun_phi));

		/*
		Set env material
		*/
		{
			OpenGLMaterial env_mat;
			env_mat.albedo_texture = opengl_engine->getTexture(base_dir + "/resources/sky_no_sun.exr");
			env_mat.albedo_texture->setTWrappingEnabled(false); // Disable wrapping in vertical direction to avoid grey dot straight up.
			
			env_mat.tex_matrix = Matrix2f(-1 / Maths::get2Pi<float>(), 0, 0, 1 / Maths::pi<float>());

			opengl_engine->setEnvMat(env_mat);
		}

		opengl_engine->setCirrusTexture(opengl_engine->getTexture(base_dir + "/resources/cirrus.exr"));


		//----------------------- Make ground plane -----------------------
		{
			GLObjectRef ground_plane = new GLObject();
			ground_plane->mesh_data = opengl_engine->getUnitQuadMeshData();
			ground_plane->ob_to_world_matrix = Matrix4f::uniformScaleMatrix(10) * Matrix4f::translationMatrix(-0.5f, -0.5f, 0);
			ground_plane->materials.resize(1);
			ground_plane->materials[0].albedo_texture = opengl_engine->getTexture(base_dir + "/resources/obstacle.png");
			ground_plane->materials[0].tex_matrix = Matrix2f(10.f, 0, 0, 10.f);

			opengl_engine->addObject(ground_plane);
		}


		TreeParams tree_params;
		tree_params.max_depth = 4;
		tree_params.trunk_base_radius = 0.045f;
		tree_params.radius_falloff_factor = 0.4f;
		tree_params.trunk_base_len = 1.f;
		tree_params.branch_len_depth_factor = 0.5f;
		tree_params.trunk_upper_radius = 0.2f;
		tree_params.trunk_noise_freq = 0.1f;
		tree_params.trunk_noise_mag = 0.0f;
		tree_params.trunk_bump_freq = 0.205f;
		tree_params.trunk_bump_mag = 0.0f;
		tree_params.tension = 0.34f;
		tree_params.phototropism_constant = 0.1f;
		tree_params.phototropism_depth_factor = 0.1f;
		tree_params.gravity_bend = 0.1f;
		
		tree_params.leaf_width = 0.06f;
		tree_params.leaf_height = 0.08f;


		GLObjectRef tree_ob;
		//----------------------- Make tree object -----------------------
		tree_ob = makeTreeObject(base_dir, tree_params, *opengl_engine);
		opengl_engine->addObject(tree_ob);
	
		//const double start_time = getCurTimeRealSec();
		int frame = 0;
		int frames_since_fps_print = 0;
		double last_fps = 0;

		Timer timer;
		Timer status_print_timer;

		float cam_phi = 0;
		float cam_theta = 1.f;
		Vec4f cam_target_pos = Vec4f(0,0,0,1);
		float cam_dist = 4;

		bool quit = false;
		while(!quit)
		{
			//const double cur_time = timer.elapsed();
			

			//TEMP:
			if(SDL_GL_MakeCurrent(win, gl_context) != 0)
			{
				std::cout << "SDL_GL_MakeCurrent failed." << std::endl;
			}


			const Matrix4f T = Matrix4f::translationMatrix(0.f, cam_dist, 0.f);
			const Matrix4f z_rot = Matrix4f::rotationMatrix(Vec4f(0,0,1,0), cam_phi);
			const Matrix4f x_rot = Matrix4f::rotationMatrix(Vec4f(1,0,0,0), -(cam_theta - Maths::pi_2<float>()));
			const Matrix4f rot = x_rot * z_rot;
			const Matrix4f world_to_camera_space_matrix = T * rot * Matrix4f::translationMatrix(-cam_target_pos);

			const float sensor_width = 0.035f;
			const float lens_sensor_dist = 0.03f;
			const float render_aspect_ratio = opengl_engine->getViewPortAspectRatio();


			int gl_w, gl_h;
			SDL_GL_GetDrawableSize(win, &gl_w, &gl_h);

			opengl_engine->setViewport(gl_w, gl_h);
			opengl_engine->setMainViewport(gl_w, gl_h);
			opengl_engine->setMaxDrawDistance(100.f);
			opengl_engine->setPerspectiveCameraTransform(world_to_camera_space_matrix, sensor_width, lens_sensor_dist, render_aspect_ratio, /*lens shift up=*/0.f, /*lens shift right=*/0.f);
			opengl_engine->setCurrentTime((float)timer.elapsed());
			opengl_engine->draw();


			ImGuiIO& imgui_io = ImGui::GetIO();

			// Draw ImGUI GUI controls
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplSDL2_NewFrame(win);
			ImGui::NewFrame();

			//ImGui::ShowDemoWindow();

			bool value_changed = false;
			// Draw horizontal slider for knob control
			value_changed = value_changed || ImGui::InputInt("max depth", &tree_params.max_depth, 1, 1);
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"trunk base radius (m)", /*val=*/&tree_params.trunk_base_radius, /*min=*/0.0f, /*max=*/1.0f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"radius_falloff_factor", /*val=*/&tree_params.radius_falloff_factor, /*min=*/0.0f, /*max=*/1.0f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"trunk_base_len", /*val=*/&tree_params.trunk_base_len, /*min=*/0.0f, /*max=*/10.0f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"branch_len_depth_factor", /*val=*/&tree_params.branch_len_depth_factor, /*min=*/0.0f, /*max=*/1.0f, "%.3f");
			//value_changed = value_changed || ImGui::SliderFloat(/*label=*/"trunk upper radius (m)", /*val=*/&tree_params.trunk_upper_radius, /*min=*/0.0f, /*max=*/1.0f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"trunk path noise freq", /*val=*/&tree_params.trunk_noise_freq, /*min=*/0.0f, /*max=*/1.0f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"trunk path noise mag", /*val=*/&tree_params.trunk_noise_mag, /*min=*/0.0f, /*max=*/1.f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"trunk bump freq", /*val=*/&tree_params.trunk_bump_freq, /*min=*/0.0f, /*max=*/1.0f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"trunk bump mag", /*val=*/&tree_params.trunk_bump_mag, /*min=*/0.0f, /*max=*/1.f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"tension", /*val=*/&tree_params.tension, /*min=*/0.0f, /*max=*/1.f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"phototropism_constant", /*val=*/&tree_params.phototropism_constant, /*min=*/0.0f, /*max=*/1.f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"phototropism_depth_factor", /*val=*/&tree_params.phototropism_depth_factor, /*min=*/0.0f, /*max=*/1.f, "%.3f");
			value_changed = value_changed || ImGui::SliderFloat(/*label=*/"gravity_bend", /*val=*/&tree_params.gravity_bend, /*min=*/0.0f, /*max=*/1.f, "%.3f");

			if(value_changed)
			{
				opengl_engine->removeObject(tree_ob);
				tree_ob = NULL;
				try
				{
					tree_ob = makeTreeObject(base_dir, tree_params, *opengl_engine);
					opengl_engine->addObject(tree_ob);
				}
				catch(glare::Exception& e)
				{
					conPrint("Error: " + e.what());
				}
			}

			
			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			// Display
			SDL_GL_SwapWindow(win);

			frame++;
			frames_since_fps_print++;

			// Handle any events
			SDL_Event e;
			while(SDL_PollEvent(&e))
			{
				if(imgui_io.WantCaptureMouse)
				{
					ImGui_ImplSDL2_ProcessEvent(&e); // Pass event onto ImGUI
					continue;
				}

				if(e.type == SDL_QUIT) // "An SDL_QUIT event is generated when the user clicks on the close button of the last existing window" - https://wiki.libsdl.org/SDL_EventType#Remarks
					quit = true;
				else if(e.type == SDL_WINDOWEVENT) // If user closes the window:
				{
					if(e.window.event == SDL_WINDOWEVENT_CLOSE)
						quit = true;
					else if(e.window.event == SDL_WINDOWEVENT_RESIZED || e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
					{
						int w, h;
						SDL_GL_GetDrawableSize(win, &w, &h);
						
						opengl_engine->setViewport(w, h);
						opengl_engine->setMainViewport(w, h);
					}
				}
				else if(e.type == SDL_KEYDOWN)
				{
					if(e.key.keysym.sym == SDLK_PLUS)
					{
					}
					else if(e.key.keysym.sym == SDLK_MINUS)
					{
					}

					if(e.key.keysym.sym == SDLK_ESCAPE) // If user presses ESC key:
						quit = true;

					if(e.key.keysym.sym >= SDLK_0 && e.key.keysym.sym <= SDLK_9)
					{
						//const int button_num = e.key.keysym.sym - SDLK_0;
					}
				}
				else if(e.type == SDL_MOUSEMOTION)
				{
					//conPrint("SDL_MOUSEMOTION");
					if(e.motion.state & SDL_BUTTON_LMASK)
					{
						//conPrint("SDL_BUTTON_LMASK down");

						const float move_scale = 0.005f;
						cam_phi += e.motion.xrel * move_scale;
						cam_theta = myClamp<float>(cam_theta - (float)e.motion.yrel * move_scale, 0.01f, Maths::pi<float>() - 0.01f);
					}

					if((e.motion.state & SDL_BUTTON_MMASK) || (e.motion.state & SDL_BUTTON_RMASK))
					{
						//conPrint("SDL_BUTTON_MMASK or SDL_BUTTON_RMASK down");

						const float move_scale = 0.005f;

						const Vec4f forwards = GeometrySampling::dirForSphericalCoords(-cam_phi + Maths::pi_2<float>(), Maths::pi<float>() - cam_theta);
						const Vec4f right = normalise(crossProduct(forwards, Vec4f(0,0,1,0)));
						const Vec4f up = crossProduct(right, forwards);

						cam_target_pos += right * -(float)e.motion.xrel * move_scale + up * (float)e.motion.yrel * move_scale;
					}
				}
				else if(e.type == SDL_MOUSEWHEEL)
				{
					//conPrint("SDL_MOUSEWHEEL");
					cam_dist = myClamp<float>(cam_dist - cam_dist * e.wheel.y * 0.2f, 0.01f, 10000.f);
				}
			}

			if(status_print_timer.elapsed() > 1)
			{
				const double fps = frames_since_fps_print / status_print_timer.elapsed();
				last_fps = fps;
				status_print_timer.reset();
				frames_since_fps_print = 0;
			}
		}
		SDL_Quit();
		return 0;
	}
	catch(glare::Exception& e)
	{
		std::cout << e.what() << std::endl;
		return 1;
	}
}
