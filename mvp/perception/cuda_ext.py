import torch
from torch import nn
from torch.autograd import Function
import sort_vertices

class SortVertices(Function):
    @staticmethod
    def forward(ctx, vertices, mask, num_valid):
        idx = sort_vertices.sort_vertices_forward(vertices, mask, num_valid)
        ctx.mark_non_differentiable(idx)
        return idx
    
    @staticmethod
    def backward(ctx, gradout):
        return ()

sort_v = SortVertices.apply

if __name__ == "__main__":
    import time
    device = torch.device('cpu')
    v = torch.rand([8, 1024, 24, 2]).float().to(device)
    mean = torch.mean(v, dim=2, keepdim=True)
    v = v - mean
    m = (torch.rand([8, 1024, 24]) > 0.8).to(device)
    nv = torch.sum(m.int(), dim=-1).int().to(device)
    start = time.time()
    result = sort_v(v, m, nv)
    print("time: %.2f ms"%((time.time() - start)*1000))
    print(result.size())
    print(result[0,0,:])