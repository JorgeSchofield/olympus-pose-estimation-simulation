function st = plant_init()
%PLANT_INIT  Estado inicial de la planta. Pose en el origen.
%#codegen
    st.x      = 0;
    st.y      = 0;
    st.th     = 0;
    st.arc    = zeros(1,6);
    st.dist   = 0;
    st.v_prev = 0;
end
