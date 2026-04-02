import { z } from "zod";
import dotenv from "dotenv";

dotenv.config();

const EnvSchema = z.object({
  JETSON_HOST: z.string().default("192.168.1.162"),
  JETSON_USER: z.string().default("timi"),
  JETSON_PORT: z.coerce.number().int().positive().default(22),
  JETSON_AUV_DIR: z.string().default("/home/timi/AUV"),
  DOCKER_CONTAINER: z.string().default("auv"),

  JETSON_PRIVATE_KEY: z.string().optional(),
  JETSON_PASSWORD: z.string().optional(),
  SSH_AUTH_SOCK: z.string().optional(),

  BACKEND_PORT: z.coerce.number().int().positive().default(8000),
  FRONTEND_ORIGIN: z.string().default("http://localhost:5173"),
});

export type AppConfig = z.infer<typeof EnvSchema>;

export function loadConfig(): AppConfig {
  const parsed = EnvSchema.safeParse(process.env);
  if (!parsed.success) {
    // Keep the message readable.
    const issues = parsed.error.issues.map((i) => `${i.path.join(".")}: ${i.message}`).join("\n");
    throw new Error(`Invalid environment:\n${issues}`);
  }
  return parsed.data;
}

