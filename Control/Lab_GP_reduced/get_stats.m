function [stats] = get_stats(f)
  dep = 0;
  % Loop over the algorithm
  for k=0:f.n_instructions()-1
    if f.instruction_id(k)==casadi.OP_CALL
      d = f.instruction_MX(k).which_function();
      if d.name()=='solver'
        dep = d;
        break
      end
    end
  end
  if dep==0
    stats = struct;
  else
    stats = dep.stats(1);
  end
end

