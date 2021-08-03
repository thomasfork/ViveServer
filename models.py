from pydantic import BaseModel, Field


class PydanticModel(BaseModel):
    valid: int = Field(default=0)
    label: str = Field(default='')
    xi: float = Field(default=0)
    xj: float = Field(default=0)
    xk: float = Field(default=0)
    qi: float = Field(default=0)
    qj: float = Field(default=0)
    qk: float = Field(default=0)
    qr: float = Field(default=1)
    v1: float = Field(default=0)
    v2: float = Field(default=0)
    v3: float = Field(default=0)
    w1: float = Field(default=0)
    w2: float = Field(default=0)
    w3: float = Field(default=0)


if __name__ == '__main__':
    pymodel = PydanticModel()
    
    import json
    
    pymsg = json.dumps(pymodel.json(), sort_keys = False).encode()
    print(len(pymsg)*100*10)
