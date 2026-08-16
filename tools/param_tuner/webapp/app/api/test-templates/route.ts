import { NextRequest, NextResponse } from "next/server";
import {
  deleteTestTemplate,
  listTestTemplates,
  saveTestTemplate,
  type TestTemplateValues,
} from "@/lib/test-templates";

export const runtime = "nodejs";

export async function GET() {
  return NextResponse.json({ templates: listTestTemplates() });
}

export async function POST(request: NextRequest) {
  const body = await request.json();
  const name = body?.name as string | undefined;
  const values = body?.values as TestTemplateValues | undefined;
  const id = body?.id as string | undefined;
  if (!name || !values) {
    return NextResponse.json({ error: "name, values is required" }, { status: 400 });
  }
  try {
    const template = saveTestTemplate(name, values, id);
    return NextResponse.json({ template });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

export async function DELETE(request: NextRequest) {
  const id = request.nextUrl.searchParams.get("id");
  if (!id) {
    return NextResponse.json({ error: "id is required" }, { status: 400 });
  }
  deleteTestTemplate(id);
  return NextResponse.json({ ok: true });
}
