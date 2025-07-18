function [p_iss_circle, s_iss_circle] = convert_cam_to_iss_frame(T_iss_cam, circle_center, surface_normal)
    p_iss_circle = T_iss_cam * [circle_center; 1];
    s_iss_circle = T_iss_cam(1:3, 1:3) * surface_normal;
    
end